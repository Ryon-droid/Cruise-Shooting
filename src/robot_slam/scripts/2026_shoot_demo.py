#!/home/abot/anaconda3/envs/py39/bin/python
# -*- coding: utf-8 -*-
import rospy
import pyaudio
import wave
import os
import subprocess
from funasr import AutoModel
import soundfile
from std_msgs.msg import String
music_path="/home/abot/8C8PE4/src/robot_slam/scripts/比赛开始.mp3"
music1_path="/home/abot/8C8PE4/src/robot_slam/scripts/提示音.mp3"
record_path="/home/abot/8C8PE4/src/robot_slam/scripts/test.wav"
os.environ['KMP_DUPLICATE_LIB_OK'] = 'True'

model = None
model_ready = False

def play_audio(audio_path):
    if not os.path.exists(audio_path):
        rospy.logerr("音频文件不存在: %s", audio_path)
        return False

    rospy.loginfo("播放音频: %s", audio_path)
    ret = subprocess.call(["mplayer", audio_path])
    if ret != 0:
        rospy.logerr("播放音频失败: %s, mplayer返回码=%s", audio_path, ret)
        return False
    return True

def audio_callback(msg):
    global model_ready
    rospy.loginfo("Received audio message, starting recording and recognition")
    if not model_ready:
        rospy.logwarn("Model not ready yet, waiting...")
        while not model_ready and not rospy.is_shutdown():
            rospy.sleep(0.1)
    start_audio()
    rospy.loginfo("Recording and recognition completed")

def start_audio(time=5, save_file="test.wav"):
    global model
    save_file = record_path
    CHUNK = 1024
    FORMAT = pyaudio.paInt16
    CHANNELS = 2
    RATE = 16000
    RECORD_SECONDS = time
    WAVE_OUTPUT_FILENAME = save_file
    try:
        p = pyaudio.PyAudio()
        rospy.loginfo("播放比赛开始提示")
        play_audio(music_path)
        rospy.loginfo("播放录音提示音")
        play_audio(music1_path)
        if os.path.exists(save_file):
            os.remove(save_file)

        stream = p.open(format=FORMAT,
                        channels=CHANNELS,
                        rate=RATE,
                        input=True,
                        frames_per_buffer=CHUNK)
        frames = []

        rospy.loginfo("ON")
        for i in range(0, int(RATE / CHUNK * RECORD_SECONDS)):
            data = stream.read(CHUNK)
            frames.append(data)
        rospy.loginfo("OFF")

        stream.stop_stream()
        stream.close()
        p.terminate()

        wf = wave.open(WAVE_OUTPUT_FILENAME, 'wb')
        wf.setnchannels(CHANNELS)
        wf.setsampwidth(p.get_sample_size(FORMAT))
        wf.setframerate(RATE)
        wf.writeframes(b''.join(frames))
        wf.close()

        rospy.loginfo("Starting recognition")
        res = model.generate(input=record_path)

        result = res[0].get('text', '默认值')
        print(result)
        rospy.loginfo("Waiting for publisher to register...")
        while pub1.get_num_connections() == 0 and not rospy.is_shutdown():
            rospy.sleep(0.1)

        message = str(result)
        rospy.loginfo("Publishing message: " + message)
        for _ in range(3):
            pub1.publish(message)
            rospy.sleep(0.2)
    except Exception as e:
        rospy.logerr("start_audio error: %s", str(e))

def audio_subscriber():
    global pub1, model, model_ready
    rospy.init_node('audio_subscriber', anonymous=True)
    rospy.loginfo("Subscribing to audio_topic...")
    rospy.Subscriber("audio_topic", String, audio_callback)
    rospy.loginfo("Successfully subscribed to audio_topic")

    pub1 = rospy.Publisher('chinese_topic', String, queue_size=10, latch=True)
    rospy.loginfo("Audio subscriber node started")

    rospy.loginfo("Loading FunASR model...")
    model = AutoModel(model="/home/abot/8C8PE4/src/robot_slam/scripts/paraformer-zh",disable_update=True)
    model_ready = True
    rospy.loginfo("FunASR model loaded, ready for recognition")

    rospy.spin()

if __name__ == '__main__':
    audio_subscriber()
