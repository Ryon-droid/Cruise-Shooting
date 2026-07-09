#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import actionlib
import rospy
import serial
from actionlib_msgs.msg import GoalStatus
from ar_track_alvar_msgs.msg import AlvarMarkers
from geometry_msgs.msg import Point, PoseWithCovarianceStamped, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Int32, String
from tf_conversions import transformations
from math import pi

# 串口配置
SERIAL_PORT = "/dev/shoot"
BAUD_RATE = 9600

# 圆形靶标参数
CIRCULAR_OBJECT_ID = 51          # 圆形目标ID
IMAGE_CENTER_X = 320             # 图像中心横坐标
CIRCULAR_TURN_THRESHOLD = 10     # 圆形靶标旋转对齐阈值（像素）
CIRCULAR_FIRE_THRESHOLD = 15     # 圆形靶标射击阈值（像素）

# 旋转靶与移动靶的偏航角对齐阈值
YAW_THRESHOLD_ROTATING = 0.1
YAW_THRESHOLD_MOVING = 0.1
TARGET_Y_RANGE = (-0.1, 0.1)     # 目标在Y方向上的允许范围


class ShootingCompetition(object):
    """射击比赛主控类"""

    def __init__(self):
        self.current_stage = "WAIT_START"      # 当前比赛阶段
        self.target_ids = {
            "rotating": None,                   # 旋转靶标ID
            "moving": None,                     # 移动靶标ID
        }
        self.fired_stages = set()               # 记录已经射击过的阶段，防止重复射击

        # 发布初始位姿
        self.set_pose_pub = rospy.Publisher(
            "/initialpose", PoseWithCovarianceStamped, queue_size=5
        )
        # 语音播报发布
        self.arrive_pub = rospy.Publisher("/voiceWords", String, queue_size=10)
        # 触发语音识别的发布
        self.audio_pub = rospy.Publisher("audio_topic", String, queue_size=10)
        # 速度控制
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1000)

        # 订阅各阶段目标ID
        rospy.Subscriber("target_id_rotating", Int32, self._rotating_id_callback)
        rospy.Subscriber("target_id_moving", Int32, self._moving_id_callback)
        # 订阅圆形靶标位置
        rospy.Subscriber("/object_position", Point, self._circular_target_handler)
        # 订阅AR标记位姿
        rospy.Subscriber("/ar_pose_marker", AlvarMarkers, self._ar_marker_handler)

        # move_base动作客户端
        self.move_base = actionlib.SimpleActionClient("move_base", MoveBaseAction)
        self.move_base.wait_for_server(rospy.Duration(60))

        # 打开串口，用于射击控制
        self.ser = serial.Serial(
            SERIAL_PORT, BAUD_RATE, parity="N", bytesize=8, stopbits=1, timeout=1
        )
        rospy.loginfo("射击串口已打开: %s", SERIAL_PORT)

    def publish_audio(self):
        """触发语音识别"""
        wait_count = 0
        # 等待音频话题有订阅者
        while (
            self.audio_pub.get_num_connections() == 0
            and wait_count < 100
            and not rospy.is_shutdown()
        ):
            rospy.sleep(0.1)
            wait_count += 1

        if rospy.is_shutdown():
            return
        self.audio_pub.publish("start_recognition")
        rospy.loginfo("已发布语音识别触发指令")

    def _rotating_id_callback(self, msg):
        """旋转靶标ID回调"""
        self.target_ids["rotating"] = msg.data
        rospy.loginfo("旋转靶标ID: %d", msg.data)

    def _moving_id_callback(self, msg):
        """移动靶标ID回调"""
        self.target_ids["moving"] = msg.data
        rospy.loginfo("移动靶标ID: %d", msg.data)

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

    def _stop_robot(self):
        """停止机器人运动"""
        self.cmd_vel_pub.publish(Twist())

    def _circular_target_handler(self, data):
        """处理圆形靶标（通过图像位置调整朝向并射击）"""
        if self.current_stage != "CIRCULAR_STAGE":
            return
        if int(data.z) != CIRCULAR_OBJECT_ID:
            return

        x_offset = data.x - IMAGE_CENTER_X
        # 如果偏移较大，旋转机器人
        if abs(x_offset) > CIRCULAR_TURN_THRESHOLD:
            self._publish_turn(-0.02 * x_offset)
            return

        # 偏移在允许范围内，停止并射击
        if abs(x_offset) <= CIRCULAR_FIRE_THRESHOLD:
            self._stop_robot()
            self._fire_once("circular")
            self.current_stage = "ROTATING_STAGE"

    def _ar_marker_handler(self, data):
        """AR标记全局回调，根据当前阶段分派处理"""
        if self.current_stage == "ROTATING_STAGE":
            self._process_rotating_target(data.markers)
        elif self.current_stage == "MOVING_STAGE":
            self._process_moving_target(data.markers)

    def _process_rotating_target(self, markers):
        """处理旋转靶标：对准后射击"""
        target_id = self.target_ids["rotating"]
        if target_id is None:
            return

        for marker in markers:
            if marker.id != target_id:
                continue

            x_pos = marker.pose.pose.position.x
            y_pos = marker.pose.pose.position.y
            # 根据x方向偏移旋转调整
            if abs(x_pos) >= YAW_THRESHOLD_ROTATING:
                self._publish_turn(-1.0 * x_pos)
            # y方向在允许范围内则射击，并切换到移动靶阶段
            elif TARGET_Y_RANGE[0] <= y_pos <= TARGET_Y_RANGE[1]:
                self._stop_robot()
                self._fire_once("rotating")
                self.current_stage = "MOVING_STAGE"
            return

    def _process_moving_target(self, markers):
        """处理移动靶标：对准后射击，结束比赛"""
        target_id = self.target_ids["moving"]
        if target_id is None:
            return

        for marker in markers:
            if marker.id != target_id:
                continue

            x_pos = marker.pose.pose.position.x
            if abs(x_pos) >= YAW_THRESHOLD_MOVING:
                self._publish_turn(-0.95 * x_pos)
            else:
                self._stop_robot()
                self._fire_once("moving")
                self.current_stage = "FINISH_STAGE"
            return

    def _navigate_to_goal(self, position, timeout=60):
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

        rospy.loginfo("开始导航至: %s", position)
        self.move_base.send_goal(goal)
        finished = self.move_base.wait_for_result(rospy.Duration(timeout))
        if not finished:
            self.move_base.cancel_goal()
            rospy.logwarn("导航超时: %s", position)
            return False

        state = self.move_base.get_state()
        if state != GoalStatus.SUCCEEDED:
            rospy.logwarn("导航未成功，状态码 %s，目标: %s", state, position)
            return False

        rospy.loginfo("已到达目标点: %s", position)
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

    def run_competition(self, goals):
        """执行整个射击比赛流程"""
        if len(goals) < 4:
            rospy.logerr("需要4个目标点: 圆形靶点, 旋转靶点, 移动靶点, 终点")
            return

        rospy.loginfo("请输入1开始射击比赛")
        # 等待用户输入开始指令
        while not rospy.is_shutdown() and self.current_stage == "WAIT_START":
            user_input = raw_input("开始？请输入1: ")
            if user_input == "1":
                self.current_stage = "CIRCULAR_STAGE"

        # 触发语音识别，获取靶标编号
        self.publish_audio()
        if not self._wait_speech_result(rospy.get_param("~speechResultTimeout", 60.0)):
            return

        # 依次前往各目标点，并在对应阶段等待完成任务
        if self._navigate_to_goal(goals[0]):
            self._wait_stage_change("CIRCULAR_STAGE")

        if self._navigate_to_goal(goals[1]):
            self._wait_stage_change("ROTATING_STAGE")

        if self._navigate_to_goal(goals[2]):
            self._wait_stage_change("MOVING_STAGE")

        self._navigate_to_goal(goals[3])
        rospy.loginfo("射击比赛完成！")


def _parse_goals():
    """从参数服务器解析目标点列表"""
    goal_x = rospy.get_param("~goalListX", "2.0,2.0,3.0,0.0")
    goal_y = rospy.get_param("~goalListY", "2.0,4.0,1.0,0.0")
    goal_yaw = rospy.get_param("~goalListYaw", "0,90,180,0")

    # 处理中文逗号
    goal_x = goal_x.replace(u"，", ",")
    goal_y = goal_y.replace(u"，", ",")
    goal_yaw = goal_yaw.replace(u"，", ",")

    goals = [
        [float(x), float(y), float(yaw)]
        for x, y, yaw in zip(goal_x.split(","), goal_y.split(","), goal_yaw.split(","))
    ]

    return goals


if __name__ == "__main__":
    rospy.init_node("shooting_competition")
    try:
        goals = _parse_goals()
        competition = ShootingCompetition()
        competition.run_competition(goals)
    except rospy.ROSInterruptException:
        pass
