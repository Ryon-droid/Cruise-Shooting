#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String, Int32
from TTS_audio.srv import StringService

target_id_rotating = None
target_id_moving = None
announced_target_ids = None

target_id_rotating_mapping = {
    "一": 1,
    "1": 1,
    "二": 2,
    "2": 2,
    "三": 3,
    "3": 3,
    "四": 4,
    "4": 4,
    "五": 5,
    "5": 5
}

target_id_moving_mapping = {
    "六": 6,
    "6": 6,
    "七": 7,
    "7": 7,
    "八": 8,
    "8": 8,
}

tts_client = None

def call_tts(text):
    global tts_client
    try:
        rospy.wait_for_service('tts_service', timeout=10)
        response = tts_client(text)
        rospy.loginfo(f"[TTS] {response.result}")
    except Exception as e:
        rospy.logerr(f"[TTS] 调用失败: {e}")

def chinese_callback(msg):
    global arrive_pub, target_id_rotating, target_id_moving, announced_target_ids

    rospy.loginfo(f"收到语音识别文本: {msg.data}")

    for keyword, value in target_id_rotating_mapping.items():
        if keyword in msg.data:
            target_id_rotating = value
            rospy.loginfo(f"检测到旋转靶关键词: {keyword}, 设置 target_id_rotating = {target_id_rotating}")
            try:
                target_id_rotating_pub.publish(target_id_rotating)
            except Exception as e:
                rospy.logerr(f"发布旋转靶 target_id 时出错: {e}")
            break

    for keyword, value in target_id_moving_mapping.items():
        if keyword in msg.data:
            target_id_moving = value
            rospy.loginfo(f"检测到移动靶关键词: {keyword}, 设置 target_id_moving = {target_id_moving}")
            try:
                target_id_moving_pub.publish(target_id_moving)
            except Exception as e:
                rospy.logerr(f"发布移动靶 target_id 时出错: {e}")
            break

    if target_id_rotating is not None and target_id_moving is not None:
        current_target_ids = (target_id_rotating, target_id_moving)
        rospy.loginfo(
            f"靶子编号发布完成: target_id_rotating={target_id_rotating}, target_id_moving={target_id_moving}"
        )
        if announced_target_ids != current_target_ids:
            announced_target_ids = current_target_ids
            arrive_str = "识别到旋转靶为{}号，移动靶为{}号".format(
                target_id_rotating,
                target_id_moving,
            )
            arrive_pub.publish(arrive_str)
            call_tts(arrive_str)

def chinese_subscriber():
    global arrive_pub, target_id_rotating_pub, target_id_moving_pub, tts_client
    rospy.init_node('chinese_subscriber', anonymous=True)

    tts_client = rospy.ServiceProxy('tts_service', StringService)

    arrive_pub = rospy.Publisher('/voiceWords', String, queue_size=10)
    target_id_rotating_pub = rospy.Publisher('target_id_rotating', Int32, queue_size=10)
    target_id_moving_pub = rospy.Publisher('target_id_moving', Int32, queue_size=10)

    rospy.Subscriber("chinese_topic", String, chinese_callback)

    rospy.loginfo("Chinese subscriber node started")
    rospy.spin()

if __name__ == '__main__':
    try:
        chinese_subscriber()
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS 节点被中断")
