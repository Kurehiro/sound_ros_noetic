#!/usr/bin/env python3
import rospy
import numpy as np
import sounddevice as sd
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


class SoundTriggerNode:
    def __init__(self):
        rospy.init_node('sound_trigger_node', anonymous=True)
        
        # --- 設定 ---
        # この周波数以上なら反応する (Hz)
        self.target_freq = rospy.get_param('~target_freq', 100.0)
        # 音量閾値
        self.vol_thresh = rospy.get_param('~vol_thresh', 0.05)
        #マイクIDの設定
        self.mic_id = rospy.get_param('~mic_id', 'default')
        
        if self.mic_id == "f":
            trigger_topic = '/first_mic/sound_trigger'
        elif self.mic_id == "s":
            trigger_topic = '/second_mic/sound_trigger'
            
        #publish topic
        self.pub_trigger = rospy.Publisher(trigger_topic, String, queue_size=10, latch=True)
        
        self.fs = rospy.get_param('~sample_rate', 44100)
        
        # デバイスIDを指定したい場合はlaunchファイル等で設定可能にする
        # 指定がなければ None (= システムのデフォルトマイクを使用)
        self.device_index = self._normalize_device_index(rospy.get_param('~device_index', None))
        self.bt_keyword = rospy.get_param('~bt_keyword', '')
        self.input_device = self._select_input_device()
        
        rospy.loginfo(f"監視開始: {self.target_freq}Hz以上の音を待っています")
        
        # デバイス情報のログ表示
        if self.input_device is None:
            rospy.loginfo(f"device: {trigger_topic}")
        else:
            rospy.loginfo(f"deviceID: {self.input_device}")
        try:
            # デフォルト（または指定ID）で接続を試みる
            # blocksize=0 はバックエンドに最適なサイズを自動決定させる設定
            with sd.InputStream(callback=self.audio_callback, 
                                channels=1,
                                device=self.input_device,
                                samplerate=self.fs, 
                                blocksize=0):
                
                rospy.loginfo("マイクーストリームを開始しました。")
                rospy.spin()
                
        except Exception as e:
            rospy.logerr(f"マイク接続エラー: {e}")
            rospy.logerr("ヒント: Bluetoothマイクが接続・入力デバイスとして有効かを確認してください。")

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
        
    def audio_callback(self, indata, frames, time_info, status):
        if status:
            print(status)
        
        data = indata[:, 0]
        
        vol = np.linalg.norm(data) / np.sqrt(len(data))
        if vol < 0.02:
            return
        
        # 2. 周波数解析 (FFT)
        window = np.hanning(len(data))
        fft_spec = np.abs(np.fft.rfft(data * window))
        fft_freq = np.fft.rfftfreq(len(data), d=1.0/self.fs)
        power_spec = fft_spec ** 2
        
        # 最も強い周波数を取得
        voice_band = np.where((fft_freq >= 100) & (fft_freq <= 4000))[0]
        noise_band = np.where((fft_freq < 80) | (fft_freq > 5000))[0]
        
        if len(voice_band) == 0:
            return

        voice_pow = np.mean(power_spec[voice_band])
        noise_pow = np.mean(power_spec[noise_band]) if len(noise_band) > 0 else 0
        
        snr_db = 10 * np.log10((voice_pow + 1e-12) / (noise_pow + 1e-12))
        
        # 3. 判定と送信
        if snr_db > 6 and voice_pow > self.vol_thresh:
            # 現在のROS時刻を取得
            word = "sound_trigger_True"
            
            # Publishする
            self.pub_trigger.publish(word)
            
            # ログ表示
            rospy.loginfo(f"voice detected! vol:{vol:.4f}, voicepow:{voice_pow:.2f}")

if __name__ == '__main__':
    SoundTriggerNode()
