#!/usr/bin/env python3
import rospy
import sounddevice as sd
import numpy as np
import scipy.io.wavfile as wav
import datetime
import threading
import os
from std_msgs.msg import String

BLUETOOTH_KEYWORDS = (
    'bluetooth',
    'bluez',
    'bluealsa',
    'a2dp',
    'hfp',
    'headset',
    'handsfree',
    'hands-free',
    'airpods',
    'jabra',
    'anker',
    'soundcore',
    'bose',
    'sony',
    'shokz',
)


class SoundSubscriberNode:
    def __init__(self):
        rospy.init_node('sound_subscriber_node', anonymous=True)
        
        self.fs = 44100
        self.channels = 1
        
        self.silence_timeout = 1.0
        self.whisper_timeout = 15.0
        
        self.is_recording = False
        self.is_waiting_whisper = False
        self.last_trigger_time = rospy.Time.now()
        self.wait_start_time = rospy.Time.now()
        
        self.recorded_frames = []
        self.lock = threading.Lock()
        
        # デバイスIDを指定したい場合はlaunchファイル等で設定可能にする
        # 指定がなければ None (= システムのデフォルトマイクを使用)
        self.device_index = self._normalize_device_index(rospy.get_param('~device_index', None))
        self.bt_keyword = rospy.get_param('~bt_keyword', '')
        self.input_device = self._select_input_device()
        
        #マイクID設定
        self.mic_id = rospy.get_param('~mic_id', 'default_mic')
        if self.mic_id == 'f':
            sub_name = '/first_mic/sound_trigger'
            pub_audio_name = '/first_mic/audio_path'
        elif self.mic_id == 's':
            sub_name = '/second_mic/sound_trigger'
            pub_audio_name = '/second_mic/audio_path'
        else:
            sub_name = f'/{self.mic_id}/sound_trigger'
            pub_audio_name = f'/{self.mic_id}/audio_path'
        
        #publish topic
        self.pub_audio = rospy.Publisher(pub_audio_name, String, queue_size=40)
        #subscribe topic
        self.sub = rospy.Subscriber(sub_name, String, self.topic_callback)
        
        rospy.loginfo("録音待機中... (トピック受信で録音開始 -> 途絶えて2秒で保存)")
        # --- マイク入力ストリームの開始 ---
        # 常に裏で音声データを受け取る（録音フラグがTrueのときだけ保存するため）
        # デバイスはシステムのデフォルト(None)を使用
        with sd.InputStream(callback=self.audio_callback,
                            channels=self.channels,
                            samplerate=self.fs,
                            device=self.input_device):
            self.loop()

    def _normalize_device_index(self, value):
        if value is None or value == '':
            return None
        try:
            return int(value)
        except (TypeError, ValueError):
            return value

    def _input_devices(self):
        try:
            devices = sd.query_devices()
            hostapis = sd.query_hostapis()
        except Exception as e:
            rospy.logwarn(f"入力デバイス一覧の取得に失敗しました: {e}")
            return []

        input_devices = []
        for index, device in enumerate(devices):
            if device.get('max_input_channels', 0) <= 0:
                continue
            hostapi_index = device.get('hostapi')
            hostapi_name = ''
            if isinstance(hostapi_index, int) and 0 <= hostapi_index < len(hostapis):
                hostapi_name = hostapis[hostapi_index].get('name', '')
            input_devices.append({
                'index': index,
                'name': device.get('name', ''),
                'hostapi': hostapi_name,
                'channels': device.get('max_input_channels', 0),
                'samplerate': device.get('default_samplerate', 0),
            })
        return input_devices

    def _log_input_devices(self, input_devices):
        if not input_devices:
            rospy.logwarn("PortAudioから見える入力デバイスがありません。")
            return
        rospy.loginfo("PortAudio入力デバイス候補:")
        for device in input_devices:
            rospy.loginfo(
                "  [{index}] {name} / hostapi={hostapi} / ch={channels} / default_sr={samplerate}".format(**device)
            )

    def _score_device(self, device, keyword=''):
        haystack = f"{device['name']} {device['hostapi']}".lower()
        score = 0
        if keyword:
            keyword_lower = keyword.lower()
            if keyword_lower in device['name'].lower():
                score += 100
            if keyword_lower in device['hostapi'].lower():
                score += 40
        for term in BLUETOOTH_KEYWORDS:
            if term in haystack:
                score += 10
        return score

    def _default_input_device(self):
        try:
            default_device = sd.default.device
            if isinstance(default_device, (list, tuple)):
                return default_device[0]
            return default_device
        except Exception:
            return None

    def _select_best_scored_device(self, input_devices, keyword=''):
        scored_devices = [
            (self._score_device(device, keyword), device)
            for device in input_devices
        ]
        scored_devices = [item for item in scored_devices if item[0] > 0]
        if not scored_devices:
            return None
        scored_devices.sort(key=lambda item: item[0], reverse=True)
        return scored_devices[0][1]

    def _select_input_device(self):
        if self.device_index is not None:
            rospy.loginfo(f"device_index指定を使用します: {self.device_index}")
            return self.device_index

        input_devices = self._input_devices()
        keyword = str(self.bt_keyword).strip()

        if keyword:
            selected = self._select_best_scored_device(input_devices, keyword)
            if selected and keyword.lower() in f"{selected['name']} {selected['hostapi']}".lower():
                rospy.loginfo(
                    f"Bluetooth入力デバイスをキーワード '{keyword}' で選択: "
                    f"[{selected['index']}] {selected['name']} ({selected['hostapi']})"
                )
                return selected['index']
            rospy.logwarn(f"キーワード '{keyword}' に一致する入力デバイスが見つかりませんでした。")
            self._log_input_devices(input_devices)

        selected = self._select_best_scored_device(input_devices)
        if selected:
            rospy.loginfo(
                f"Bluetooth入力デバイスを自動選択: "
                f"[{selected['index']}] {selected['name']} ({selected['hostapi']})"
            )
            return selected['index']

        rospy.logwarn("Bluetooth入力デバイス候補が見つかりませんでした。OS既定入力デバイスを使用します。")
        self._log_input_devices(input_devices)
        return self._default_input_device()
    
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
            rate.sleep()

if __name__ == '__main__':
    try:
        SoundSubscriberNode()
    except rospy.ROSInterruptException:
        pass
