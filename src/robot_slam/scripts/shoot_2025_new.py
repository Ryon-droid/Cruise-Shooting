#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import actionlib
import rospy
import serial
import math
from actionlib_msgs.msg import GoalStatus
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int32, String
from tf_conversions import transformations
from math import pi

# 串口配置
SERIAL_PORT = "/dev/shoot"
BAUD_RATE = 9600

# 圆形靶标参数
CIRCULAR_OBJECT_ID = 35          # 静止圆形目标ID
IMAGE_CENTER_X = 320             # 枪口对准时的目标中心横坐标
CIRCULAR_TURN_THRESHOLD = 6      # 圆形靶标旋转对齐阈值（像素），偏差超过该值就继续调整
CIRCULAR_FIRE_THRESHOLD = 6      # 圆形靶标射击阈值（像素）
CIRCULAR_STABLE_FRAMES = 4       # 连续稳定帧数，避免靠运气瞬间开火
CIRCULAR_FORCE_FIRE_TIMEOUT = 15.0
CIRCULAR_TURN_GAIN = 0.012       # 静止靶像素偏差转角速度增益
CIRCULAR_TURN_MAX = 1.20         # 静止靶最大角速度，避免一次转过头
CIRCULAR_TURN_MIN = 0.16         # 静止靶最小有效角速度，克服底盘死区
CIRCULAR_TURN_PULSE_TIME = 0.35  # 静止靶识别刷新慢时，连续发速度的时间

# 官方例程旋转靶与移动靶火控参数
YAW_THRESHOLD_ROTATING = 0.03
YAW_THRESHOLD_MOVING = 0.03
TARGET_Y_RANGE = (-0.22, -0.10)
ROTATING_FIRE_X_OFFSET = 0.03   # 旋转靶开火/跟踪补偿；打左偏就增大，打右偏就减小
MOVING_FIRE_X_LEAD = 0.05       # 平移靶动态提前量；打慢就增大，打早就减小
MOVING_FIRE_MIN_DELTA = 0.005   # 判断平移靶运动方向的最小x变化，过滤识别抖动
ROTATING_AR_TRACK_KPZ = 3.0
MOVING_AR_TRACK_KPZ = 1.0
AR_TRACK_MAX_VEL_Z = 0.5
AR_TRACK_THRES_VEL_Z = 0.05
ROTATING_AR_X_OFFSET = 0.1
MOVING_AR_X_OFFSET = 0.165
ROTATING_Y_CENTER = -0.47
ROTATING_Y_TOLERANCE = 0.06
MOVING_Y_CENTER = -0.25
MOVING_Y_TOLERANCE = 0.04
ROTATING_STABLE_FRAMES = 1
MOVING_STABLE_FRAMES = 1
FIRE_SETTLE_TIME = 0.30
SAFE_STOP_DISTANCE = 0.16        # 开环移动时，运动方向上的最小激光距离


class ShootingCompetition(object):
    """射击比赛主控类"""

    def __init__(self):
        self.current_stage = "WAIT_START"      # 当前比赛阶段
        self.target_ids = {
            "rotating": None,                   # 旋转靶标ID
            "moving": None,                     # 移动靶标ID
        }
        self.fired_stages = set()               # 记录已经射击过的阶段，防止重复射击
        self.circular_stable_count = 0
        self.rotating_stable_count = 0
        self.moving_stable_count = 0
        self.last_moving_raw_x = None
        self.scan_msg = None
        self.manual_input_locked = False
        self.circular_aiming_enabled = False
        self.aiming_enabled = False
        self.stage_start_time = rospy.Time.now()

        # 发布初始位姿
        self.set_pose_pub = rospy.Publisher(
            "/initialpose", PoseWithCovarianceStamped, queue_size=5
        )
        # 语音播报发布
        self.arrive_pub = rospy.Publisher("/voiceWords", String, queue_size=10)
        # 触发语音识别的发布（保留但不再使用，可删除或保留兼容）
        self.audio_pub = rospy.Publisher("audio_topic", String, queue_size=10)
        # 速度控制
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1000)

        # 订阅各阶段目标ID（语音识别仍可运行，但不会被使用）
        rospy.Subscriber("target_id_rotating", Int32, self._rotating_id_callback)
        rospy.Subscriber("target_id_moving", Int32, self._moving_id_callback)
        # 订阅圆形靶标位置
        rospy.Subscriber("/object_position", Point, self._circular_target_handler)
        # 订阅AR标记位姿
        rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self._ar_marker_handler)
        # 开环后退/横移也要看激光，避免直接撞围挡
        rospy.Subscriber("/scan_filtered", LaserScan, self._scan_callback)

        # move_base动作客户端
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(60))

        # 打开串口，用于射击控制
        self.ser = serial.Serial(
            SERIAL_PORT, BAUD_RATE, parity="N", bytesize=8, stopbits=1, timeout=1
        )
        rospy.loginfo("射击串口已打开: %s", SERIAL_PORT)

    def publish_audio(self):
        """
        语音识别触发函数
        发布音频指令到指定话题
        """
        wait_deadline = rospy.Time.now() + rospy.Duration(10.0)
        while (
            self.audio_pub.get_num_connections() == 0
            and not rospy.is_shutdown()
            and rospy.Time.now() < wait_deadline
        ):
            rospy.loginfo_throttle(1.0, "等待录音识别节点连接 audio_topic...")
            rospy.sleep(0.1)

        publish_count = 1
        while not rospy.is_shutdown() and publish_count > 0:
            audio_data = "start_recognition"  # 启动语音识别指令
            self.audio_pub.publish(audio_data)
            rospy.loginfo("已发送语音识别指令: %s", audio_data)
            publish_count -= 1
            rate = rospy.Rate(1)
            rate.sleep()

    def _rotating_id_callback(self, msg):
        """旋转靶标ID回调"""
        if self.manual_input_locked:
            return
        self.target_ids["rotating"] = msg.data
        rospy.loginfo("旋转靶标ID: %d", msg.data)

    def _moving_id_callback(self, msg):
        """移动靶标ID回调"""
        if self.manual_input_locked:
            return
        self.target_ids["moving"] = msg.data
        self.last_moving_raw_x = None
        rospy.loginfo("移动靶标ID: %d", msg.data)

    def _get_neighbor_marker_pair(self, markers, target_id):
        marker_by_id = {marker.id: marker for marker in markers}
        left = marker_by_id.get(target_id - 1)
        right = marker_by_id.get(target_id + 1)
        if left is not None and right is not None:
            return left, right
        return None

    def _fire_once(self, stage):
        """向指定阶段的目标射击（同一阶段只射击一次）"""
        if stage in self.fired_stages:
            return
        self.fired_stages.add(stage)

        # 发送射击指令（示例中的串口协议）
        self.ser.write(b"\x55\x01\x12\x00\x00\x00\x01\x69")
        rospy.sleep(0.1)
        self.ser.write(b"\x55\x01\x11\x00\x00\x00\x01\x68")
        rospy.loginfo("已向%s靶标开火", stage)

    def _publish_turn(self, angular_z):
        """发布旋转指令，调整朝向"""
        twist = Twist()
        twist.angular.z = angular_z
        self.cmd_vel_pub.publish(twist)

    def _set_stage(self, stage):
        self.current_stage = stage
        self.circular_aiming_enabled = False
        self.aiming_enabled = False
        self.stage_start_time = rospy.Time.now()
        if stage == "MOVING_STAGE":
            self.last_moving_raw_x = None

    def _publish_turn_pulse(self, angular_z, duration=0.12):
        """短脉冲旋转，避免识别刷新慢时持续转向过冲。"""
        twist = Twist()
        twist.angular.z = angular_z
        end_time = rospy.Time.now() + rospy.Duration(duration)
        rate = rospy.Rate(30)
        while not rospy.is_shutdown() and rospy.Time.now() < end_time:
            self.cmd_vel_pub.publish(twist)
            rate.sleep()
        self._stop_robot()

    def _stop_robot(self):
        """停止机器人运动"""
        self.cmd_vel_pub.publish(Twist())

    def _stop_and_settle(self, duration=FIRE_SETTLE_TIME):
        """开火前连续发停止指令，等底盘角速度降下来。"""
        end_time = rospy.Time.now() + rospy.Duration(duration)
        rate = rospy.Rate(50)
        while not rospy.is_shutdown() and rospy.Time.now() < end_time:
            self._stop_robot()
            rate.sleep()

    def _satfunc(self, data, max_value, threshold):
        if abs(data) < threshold:
            return 0.0
        if abs(data) > max_value:
            return max_value if data > 0 else -max_value
        return data

    def _ar_track_turn_speed(self, x_pos, kpz):
        return self._satfunc(-kpz * x_pos, AR_TRACK_MAX_VEL_Z, AR_TRACK_THRES_VEL_Z)

    def _scan_callback(self, msg):
        self.scan_msg = msg

    def _min_scan_in_sector(self, center_angle, half_width):
        if self.scan_msg is None:
            return None

        values = []
        angle = self.scan_msg.angle_min
        for distance in self.scan_msg.ranges:
            wrapped = math.atan2(math.sin(angle - center_angle), math.cos(angle - center_angle))
            if abs(wrapped) <= half_width and not math.isnan(distance) and not math.isinf(distance) and distance > 0.02:
                values.append(distance)
            angle += self.scan_msg.angle_increment
        return min(values) if values else None

    def _too_close_to_wall(self, linear_x=0.0, linear_y=0.0):
        if abs(linear_x) >= abs(linear_y):
            center_angle = 0.0 if linear_x >= 0 else math.pi
            sector = "前方" if linear_x >= 0 else "后方"
        else:
            center_angle = math.pi / 2.0 if linear_y >= 0 else -math.pi / 2.0
            sector = "左侧" if linear_y >= 0 else "右侧"

        distance = self._min_scan_in_sector(center_angle, math.radians(18.0))
        if distance is not None and distance < SAFE_STOP_DISTANCE:
            return True, distance, sector
        return False, distance, sector

    def _circular_target_handler(self, data):
        """处理圆形靶标（通过图像位置调整朝向并射击）"""
        if self.current_stage != "CIRCULAR_STAGE":
            rospy.loginfo_throttle(2.0, "收到静止靶数据，但当前阶段=%s，未进入静止靶调整", self.current_stage)
            return
        if not self.circular_aiming_enabled:
            rospy.loginfo_throttle(2.0, "收到静止靶数据，但静止靶瞄准未开启")
            return
        if int(data.z) != CIRCULAR_OBJECT_ID:
            self._stop_robot()
            rospy.loginfo_throttle(1.0, "静止靶识别到ID=%d，等待ID=%d", int(data.z), CIRCULAR_OBJECT_ID)
            return

        x_offset = data.x - IMAGE_CENTER_X
        elapsed = (rospy.Time.now() - self.stage_start_time).to_sec()
        rospy.loginfo_throttle(0.5, "静止靶ID=%d，x偏差=%.1f，稳定帧=%d/%d",
                               int(data.z), x_offset, self.circular_stable_count, CIRCULAR_STABLE_FRAMES)
        if elapsed >= CIRCULAR_FORCE_FIRE_TIMEOUT:
            self._stop_robot()
            rospy.logwarn("静止靶调整超过%.1fs，当前x偏差=%.1f，直接开火", CIRCULAR_FORCE_FIRE_TIMEOUT, x_offset)
            self._fire_once("circular")
            self._set_stage("ROTATING_STAGE")
            return
        # 如果偏移较大，旋转机器人
        if abs(x_offset) > CIRCULAR_TURN_THRESHOLD:
            self.circular_stable_count = 0
            turn_speed = max(min(-CIRCULAR_TURN_GAIN * x_offset, CIRCULAR_TURN_MAX), -CIRCULAR_TURN_MAX)
            if abs(turn_speed) < CIRCULAR_TURN_MIN:
                turn_speed = CIRCULAR_TURN_MIN if turn_speed > 0 else -CIRCULAR_TURN_MIN

            rospy.loginfo_throttle(
                0.5,
                "静止靶连续火控调整: x偏差=%.1f, angular.z=%.3f, pulse=%.2fs",
                x_offset,
                turn_speed,
                CIRCULAR_TURN_PULSE_TIME,
            )
            self._publish_turn_pulse(turn_speed, CIRCULAR_TURN_PULSE_TIME)
            return

        if abs(x_offset) <= CIRCULAR_FIRE_THRESHOLD:
            self._stop_robot()
            self.circular_stable_count += 1
            if self.circular_stable_count >= CIRCULAR_STABLE_FRAMES:
                self._fire_once("circular")
                self._set_stage("ROTATING_STAGE")

    def _ar_marker_handler(self, data):
        """AR标记全局回调，根据当前阶段分派处理"""
        if not self.aiming_enabled:
            return
        if self.current_stage == "ROTATING_STAGE":
            self._process_rotating_target(data.markers)
        elif self.current_stage == "MOVING_STAGE":
            self._process_moving_target(data.markers)

    def _process_rotating_target(self, markers):
        """处理旋转靶逻辑（官方例程）"""
        target_id = self.target_ids["rotating"]
        if target_id is None:
            return

        for marker in markers:
            if marker.id == target_id:
                raw_x = marker.pose.pose.position.x
                x_pos = raw_x + ROTATING_FIRE_X_OFFSET
                y_pos = marker.pose.pose.position.y

                if abs(x_pos) >= YAW_THRESHOLD_ROTATING:
                    twist = Twist()
                    twist.angular.z = -1 * x_pos
                    rospy.loginfo_throttle(
                        0.5,
                        "旋转靶官方火控调整: raw_x=%.3f, 补偿x=%.3f, y=%.3f, angular.z=%.3f",
                        raw_x,
                        x_pos,
                        y_pos,
                        twist.angular.z,
                    )
                    self.cmd_vel_pub.publish(twist)
                elif TARGET_Y_RANGE[0] <= y_pos <= TARGET_Y_RANGE[1]:
                    self._stop_robot()
                    self._fire_once("rotating")
                    self._set_stage("MOVING_STAGE")
                else:
                    self._stop_robot()
                    rospy.loginfo_throttle(
                        0.5,
                        "旋转靶x已对准但y超范围: raw_x=%.3f, 补偿x=%.3f, y=%.3f, 需要 %.2f~%.2f",
                        raw_x,
                        x_pos,
                        y_pos,
                        TARGET_Y_RANGE[0],
                        TARGET_Y_RANGE[1],
                    )
                return

    def _process_moving_target(self, markers):
        """处理移动靶逻辑（官方例程）"""
        target_id = self.target_ids["moving"]
        if target_id is None:
            return

        for marker in markers:
            if marker.id == target_id:
                raw_x = marker.pose.pose.position.x
                delta_x = 0.0
                x_pos = raw_x

                if self.last_moving_raw_x is not None:
                    delta_x = raw_x - self.last_moving_raw_x
                    if abs(delta_x) >= MOVING_FIRE_MIN_DELTA:
                        if delta_x > 0:
                            x_pos = raw_x + MOVING_FIRE_X_LEAD
                        else:
                            x_pos = raw_x - MOVING_FIRE_X_LEAD
                self.last_moving_raw_x = raw_x

                if abs(x_pos) >= YAW_THRESHOLD_MOVING:
                    twist = Twist()
                    twist.angular.z = -0.95 * x_pos
                    rospy.loginfo_throttle(
                        0.5,
                        "移动靶动态火控调整: raw_x=%.3f, dx=%.3f, 补偿x=%.3f, angular.z=%.3f",
                        raw_x,
                        delta_x,
                        x_pos,
                        twist.angular.z,
                    )
                    self.cmd_vel_pub.publish(twist)
                else:
                    self._fire_once("moving")
                    self._set_stage("FINISH_STAGE")
                return

    def _navigate_to_goal(self, position, timeout=60, label="目标点"):
        """导航到指定目标点（x, y, yaw角度）"""
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = position[0]
        goal.target_pose.pose.position.y = position[1]

        # 将欧拉角转换为四元数
        q = transformations.quaternion_from_euler(0.0, 0.0, position[2] / 180.0 * pi)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]

        rospy.loginfo("开始导航至%s: %s", label, position)
        self.move_base.send_goal(goal)
        finished = self.move_base.wait_for_result(rospy.Duration(timeout))
        if not finished:
            self.move_base.cancel_goal()
            rospy.logwarn("%s导航超时: %s", label, position)
            return False

        state = self.move_base.get_state()
        if state != GoalStatus.SUCCEEDED:
            rospy.logwarn("%s导航未成功，状态码 %s，目标: %s", label, state, position)
            return False

        rospy.loginfo("已到达%s: %s", label, position)
        return True

    def _wait_stage_change(self, stage):
        """等待当前阶段发生变化"""
        rate = rospy.Rate(10)
        while not rospy.is_shutdown() and self.current_stage == stage:
            rate.sleep()
    def _wait_speech_result(self, timeout=60.0):
        """等待语音识别返回靶标ID"""
        start_time = rospy.Time.now()
        rate = rospy.Rate(10)
        rospy.loginfo("等待语音识别返回靶标编号……")
        while not rospy.is_shutdown():
            if (
                self.target_ids["rotating"] is not None
                and self.target_ids["moving"] is not None
            ):
                rospy.loginfo(
                    "语音识别完成: 旋转靶ID=%d, 移动靶ID=%d",
                    self.target_ids["rotating"],
                    self.target_ids["moving"],
                )
                return True

            if (rospy.Time.now() - start_time).to_sec() > timeout:
                rospy.logerr(
                    "语音识别超时: 旋转靶ID=%s, 移动靶ID=%s",
                    self.target_ids["rotating"],
                    self.target_ids["moving"],
                )
                return False

            rate.sleep()
    def _manual_input_ids(self):
        """手动从终端输入靶标ID（替代语音识别）"""
        import sys
        # Python2/3 兼容
        if sys.version_info[0] < 3:
            input_func = raw_input
        else:
            input_func = input

        while not rospy.is_shutdown():
            try:
                rotating_id_str = input_func("请输入旋转靶标ID: ")
                moving_id_str = input_func("请输入移动靶标ID: ")
                rotating_id = int(rotating_id_str)
                moving_id = int(moving_id_str)
                self.target_ids["rotating"] = rotating_id
                self.target_ids["moving"] = moving_id
                self.manual_input_locked = True
                rospy.loginfo("手动输入完成: 旋转靶ID=%d, 移动靶ID=%d",
                              rotating_id, moving_id)
                return True
            except (ValueError, EOFError):
                rospy.logwarn("输入无效，请重新输入整数ID")

    def _move_distance(self, linear_x=0.18, duration=5.0, check_obstacle=True):
        """直接控制轮子移动指定时间（替代导航）"""
        rospy.loginfo("开始直接移动，速度=%.2fm/s，时间=%.2fs，避障=%s",
                      linear_x, duration, check_obstacle)
        
        # 创建速度指令
        twist = Twist()
        twist.linear.x = linear_x
        twist.angular.z = 0.0
        
        # 发布速度指令
        start_time = rospy.Time.now().to_sec()
        rate = rospy.Rate(50)  # 50Hz发布频率
        
        while not rospy.is_shutdown():
            elapsed = rospy.Time.now().to_sec() - start_time
            if elapsed >= duration:
                break
            if check_obstacle:
                too_close, distance, sector = self._too_close_to_wall(linear_x=linear_x)
                if too_close:
                    rospy.logwarn("%s激光距离 %.2fm，停止开环直线移动", sector, distance)
                    break
            self.cmd_vel_pub.publish(twist)
            rate.sleep()
        self._stop_robot()

    def _move_lateral_omni(self, distance=1.0, linear_speed=0.18):
        """全向底盘直接横移（设置 linear.y）"""
        rospy.loginfo("全向底盘横移，距离=%.2fm", distance)
        
        twist = Twist()
        twist.linear.y = -linear_speed  # 负值向右，正值向左
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        
        duration = abs(distance) / linear_speed
        start_time = rospy.Time.now().to_sec()
        rate = rospy.Rate(50)
        
        while not rospy.is_shutdown():
            elapsed = rospy.Time.now().to_sec() - start_time
            if elapsed >= duration:
                break
            too_close, distance, sector = self._too_close_to_wall(linear_y=twist.linear.y)
            if too_close:
                rospy.logwarn("%s激光距离 %.2fm，停止开环横移", sector, distance)
                break
            self.cmd_vel_pub.publish(twist)
            rate.sleep()
        
        self._stop_robot()
        rospy.loginfo("横移完成")

    def _move_until_obstacle(self, linear_x=0.0, linear_y=0.0, timeout=8.0):
        """沿指定方向移动，直到激光检测到障碍物或超时。"""
        twist = Twist()
        twist.linear.x = linear_x
        twist.linear.y = linear_y
        rospy.loginfo("开始移动直到障碍物，linear.x=%.2f, linear.y=%.2f, 超时=%.1fs",
                      linear_x, linear_y, timeout)

        start_time = rospy.Time.now().to_sec()
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            elapsed = rospy.Time.now().to_sec() - start_time
            if elapsed >= timeout:
                rospy.logwarn("移动直到障碍物超时 %.1fs，停止", timeout)
                break
            too_close, distance, sector = self._too_close_to_wall(linear_x=linear_x, linear_y=linear_y)
            if too_close:
                rospy.logwarn("%s激光距离 %.2fm，已到障碍物，停止", sector, distance)
                break
            self.cmd_vel_pub.publish(twist)
            rate.sleep()
        self._stop_robot()

    def run_competition(self, goals, trans_points):
        """执行整个射击比赛流程"""
        if len(goals) < 4:
            rospy.logerr("需要4个目标点: 圆形靶点, 旋转靶点, 移动靶点, 终点")
            return

        rospy.loginfo("请输入1开始射击比赛")
        # 等待用户输入开始指令
        while not rospy.is_shutdown() and self.current_stage == "WAIT_START":
            user_input = raw_input("开始?请输入1: ")
            if user_input == "1":
                self.current_stage = "audio_STAGE"

        self.publish_audio()
        if not self._wait_speech_result(rospy.get_param("~speechResultTimeout", 60.0)):
            return

        self.current_stage = "CIRCULAR_STAGE"

        # 依次前往各目标点，并在对应阶段等待完成任务
        if not self._navigate_to_goal(goals[0], label="静止靶射击点"):
            rospy.logwarn("圆形靶点导航失败，停在当前位置继续等待静止靶射击")
            self._stop_robot()
        self.move_base.cancel_all_goals()
        self._stop_robot()
        rospy.sleep(0.2)
        self.circular_aiming_enabled = True
        self.stage_start_time = rospy.Time.now()
        self._wait_stage_change("CIRCULAR_STAGE")
        self.circular_aiming_enabled = False

        #if not self._navigate_to_goal(trans_points[0]):
            #rospy.logwarn("过渡点1导航失败,跳过")   
        self._move_distance(linear_x=-0.14, duration=7.0, check_obstacle=False)
        if not self._navigate_to_goal(trans_points[1], label="过渡点2"):
            rospy.logwarn("过渡点2导航失败,跳过")

        
        #self._move_lateral_omni(distance=1.0, linear_speed=0.25)

        if self._navigate_to_goal(goals[1], label="旋转靶射击点"):
            self.aiming_enabled = True
            self._wait_stage_change("ROTATING_STAGE")
            self.aiming_enabled = False

        self._move_distance(linear_x=-0.15, duration=7.0, check_obstacle=False)
        if not self._navigate_to_goal(trans_points[2], label="过渡点3"):
            rospy.logwarn("过渡点3导航失败,跳过")

        if not self._navigate_to_goal(trans_points[3], label="过渡点4"):
            rospy.logwarn("过渡点4导航失败,跳过")

        if self._navigate_to_goal(goals[2], label="平移靶射击点"):
            self.aiming_enabled = True
            self._wait_stage_change("MOVING_STAGE")
            self.aiming_enabled = False

        # 直接移动到终点（替代导航）
        rospy.loginfo("移动靶射击完成，先后退7.0秒，再向右平移到障碍物，最长2.1秒")
        self._move_distance(linear_x=-0.15, duration=7.0, check_obstacle=False)
        self._move_until_obstacle(linear_y=-0.14, timeout=2.1)

        rospy.loginfo("射击比赛完成！")


def _parse_goals():
    """从参数服务器解析目标点列表"""
    goal_x = rospy.get_param("~goalListX", "0.0,0.0,0.0,0.0")
    goal_y = rospy.get_param("~goalListY", "0.0,0.0,0.0,0.0")
    goal_yaw = rospy.get_param("~goalListYaw", "0,0,0,0")
    # 过渡点参数（在目标点之间添加）
    trans_x = rospy.get_param("~transX", "0.0,0.0,0.0,0.0")
    trans_y = rospy.get_param("~transY", "0.0,0.0,0.0,0.0")
    trans_yaw = rospy.get_param("~transYaw", "0,0,0,0")

    # 处理中文逗号
    goal_x = goal_x.replace(u"，", ",")
    goal_y = goal_y.replace(u"，", ",")
    goal_yaw = goal_yaw.replace(u"，", ",")
    trans_x = trans_x.replace(u"，", ",")
    trans_y = trans_y.replace(u"，", ",")
    trans_yaw = trans_yaw.replace(u"，", ",")

    goals = [
        [float(x.strip().replace(" ", "")), float(y.strip().replace(" ", "")), float(yaw.strip().replace(" ", ""))]
        for x, y, yaw in zip(goal_x.split(","), goal_y.split(","), goal_yaw.split(","))
    ]

    trans_points = [
        [float(x.strip().replace(" ", "")), float(y.strip().replace(" ", "")), float(yaw.strip().replace(" ", ""))]
        for x, y, yaw in zip(trans_x.split(","), trans_y.split(","), trans_yaw.split(","))
    ]

    return goals, trans_points


if __name__ == "__main__":
    rospy.init_node("shooting_competition")
    try:
        goals, trans_points = _parse_goals()
        competition = ShootingCompetition()
        competition.run_competition(goals, trans_points)
    except rospy.ROSInterruptException:
        pass
