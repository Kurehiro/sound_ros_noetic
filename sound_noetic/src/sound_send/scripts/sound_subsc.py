#!/usr/bin/env python3
import rospy
import sounddevice as sd
import numpy as np
import scipy.io.wavfile as wav
import datetime
import threading
import os
from std_msgs.msg import String

class SoundSubscriberNode:
    def __init__(self):
        rospy.init_node('sound_subscriber_node', anonymous=True)
        
        self.fs = 44100
        self.channels = 1
        self.silence_timeout = 2.0
        
        self.is_recording = False
        self.is_waiting_whisper = False
        self.last_trigger_time = rospy.Time.now()
        self.recorded_frames = []
        self.lock = threading.Lock()
        
        self.sub = rospy.Subscriber('/sound_trigger', String, self.topic_callback)
        self.pub_audio = rospy.Publisher('/audio_path', String, queue_size=10)
        self.sub_state = rospy.Subscriber('/whisper_state', String, self.state_callback)
        
        rospy.loginfo("録音待機中... (トピック受信で録音開始 -> 途絶えて2秒で保存)")
        # --- マイク入力ストリームの開始 ---
        # 常に裏で音声データを受け取る（録音フラグがTrueのときだけ保存するため）
        # デバイスはシステムのデフォルト(None)を使用
        with sd.InputStream(callback=self.audio_callback,
                            channels=self.channels,
                            samplerate=self.fs,
                            device=None):
            self.loop()
    
    def topic_callback(self, msg):
        """
        /sound_trigger トピックを受け取り時の処理
        """
        with self.lock:
            if not self.is_recording and not self.is_waiting_whisper:
                self.last_trigger_time = rospy.Time.now()
                rospy.loginfo("trigger on: record start")
                self.is_recording = True
                self.recorded_frames = []
    
    def state_callback(self, msg):
        if msg.data == "done":
            self.is_waiting_whisper = False
            rospy.loginfo("whisper succeeded: next")
    
    def audio_callback(self, indata, frames, time, status):
        """
        音声入力スレッド
        """
        if status:
            print(status)
        with self.lock:
            if self.is_recording:
                self.recorded_frames.append(indata.copy())
    
    def save_audio(self):
        """
        音声データをwavファイルに保存
        """
        if not self.recorded_frames:
            return
        
        filename = datetime.datetime.now().strftime("task_sound.wav")
        full_data = np.concatenate(self.recorded_frames, axis=0)
        wav.write(filename, self.fs, full_data)
        
        abs_path = os.path.abspath(filename)
        
        rospy.loginfo(f"record stop: {filename}")
        self.pub_audio.publish(abs_path)
        
        self.is_waiting_whisper = True
        rospy.loginfo("Entering whisper response waiting mode")
    
    def loop(self):
        """
        メインループ:タイムアウトの監視
        """
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            with self.lock:
                if self.is_recording:
                    elapsed = (rospy.Time.now() - self.last_trigger_time).to_sec()
                    if elapsed > self.silence_timeout:
                        self.save_audio()
                        self.is_recording = False
                        self.recorded_frames = []
            rate.sleep()

if __name__ == '__main__':
    try:
        SoundSubscriberNode()
    except rospy.ROSInterruptException:
        pass