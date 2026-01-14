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
        
        self.silence_timeout = 1.0
        self.whisper_timeout = 15.0
        
        self.is_recording = False
        self.last_trigger_time = rospy.Time.now()
        
        self.recorded_frames = []
        self.lock = threading.Lock()
        
        # デバイスIDを指定したい場合はlaunchファイル等で設定可能にする
        # 指定がなければ None (= システムのデフォルトマイクを使用)
        self.device_index = rospy.get_param('~device_index', None)
        
        #マイクID設定
        self.mic_id = rospy.get_param('~mic_id', 'default_mic')
        if self.mic_id == 'f':
            sub_name = '/first_mic/sound_trigger'
            pub_audio_name = '/first_mic/audio_path'
            sub_name = '/second_mic/sound_trigger'
            pub_audio_name = '/second_mic/audio_path'
        else:
            sub_name = f'/{self.mic_id}/sound_trigger'
            pub_audio_name = f'/{self.mic_id}/audio_path'
            sub_name = f'/{self.mic_id}/sound_trigger'
            pub_audio_name = f'/{self.mic_id}/audio_path'
        
        #publish topic
        self.pub_audio = rospy.Publisher(pub_audio_name, String, queue_size=10)
        #subscribe topic
        self.sub = rospy.Subscriber(sub_name, String, self.topic_callback)
        self.sub = rospy.Subscriber(sub_name, String, self.topic_callback)
        
        rospy.loginfo("録音待機中... (トピック受信で録音開始 -> 途絶えて2秒で保存)")
        # --- マイク入力ストリームの開始 ---
        # 常に裏で音声データを受け取る（録音フラグがTrueのときだけ保存するため）
        # デバイスはシステムのデフォルト(None)を使用
        with sd.InputStream(callback=self.audio_callback,
                            channels=self.channels,
                            samplerate=self.fs,
                            device=self.device_index):
            self.loop()
    
    def topic_callback(self, msg):
        """
        /sound_trigger トピックを受け取り時の処理
        - 重要: トリガーは常に last_trigger_time を更新して録音延長させる
        """
        rospy.loginfo(f"DEBUG: topic subscribe (Msg: {msg.data})")
        with self.lock:
            now = rospy.Time.now()
            # 常に更新して録音を延長可能にする
            self.last_trigger_time = now

            if not self.is_recording:
                self.is_recording = True
                self.recorded_frames = []
                rospy.loginfo("trigger on: record start")
            else:
                # 録音中のトリガーは録音継続（last_trigger_time を更新したことで実現）
                rospy.loginfo("trigger: extend recording")

    
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
        
        filename = datetime.datetime.now().strftime("task_sound_%Y%m%d_%H%M%S.wav")
        full_data = np.concatenate(self.recorded_frames, axis=0)
        wav.write(filename, self.fs, full_data)
        
        abs_path = os.path.abspath(filename)
        
        rospy.loginfo(f"record stop: {filename}")
        self.pub_audio.publish(abs_path)
    
    def loop(self):
        """
        メインループ:タイムアウトの監視
        """
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            with self.lock:
                current_time = rospy.Time.now()
                if self.is_recording:
                    elapsed = (rospy.Time.now() - self.last_trigger_time).to_sec()
                    if elapsed > self.silence_timeout:
                        self.save_audio()
                        self.is_recording = False
                        self.recorded_frames = []
                        self.recorded_frames = []
            rate.sleep()

if __name__ == '__main__':
    try:
        SoundSubscriberNode()
    except rospy.ROSInterruptException:
        pass