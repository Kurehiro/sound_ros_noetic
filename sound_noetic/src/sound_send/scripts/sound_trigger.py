#!/usr/bin/env python3
import rospy
import numpy as np
import sounddevice as sd
from std_msgs.msg import String

class SoundTriggerNode:
    def __init__(self):
        rospy.init_node('sound_trigger_node', anonymous=True)
        
        # --- 設定 ---
        # この周波数以上なら反応する (Hz)
        self.target_freq = rospy.get_param('~target_freq', 300.0)
        # 音量閾値
        self.vol_thresh = rospy.get_param('~vol_thresh', 0.2)
        #マイクIDの設定
        self.mic_id = rospy.get_param('~mic_id', 'default')
        
        if self.mic_id == "f":
            trigger_topic = '/first_mic/sound_trigger'
        elif self.mic_id == "s":
            trigger_topic = '/second_mic/sound_trigger'
            
        #publish topic
        self.pub_trigger = rospy.Publisher(trigger_topic, String, queue_size=10, latch=False)
        
        self.fs = rospy.get_param('~sample_rate', 44100)
        
        self._last_pub_time = rospy.Time(0)
        self._min_pub_interval = rospy.get_param('~min_val', 0.05)
        
        
        # デバイスIDを指定したい場合はlaunchファイル等で設定可能にする
        # 指定がなければ None (= システムのデフォルトマイクを使用)
        self.device_index = rospy.get_param('~device_index', None) 
        
        rospy.loginfo(f"監視開始: {self.target_freq}Hz以上の音を待っています")
        
        # デバイス情報のログ表示
        if self.device_index is None:
            rospy.loginfo(f"device: {trigger_topic}")
        else:
            rospy.loginfo(f"deviceID: {self.device_index}")
        try:
            # デフォルト（または指定ID）で接続を試みる
            # blocksize=0 はバックエンドに最適なサイズを自動決定させる設定
            with sd.InputStream(callback=self.audio_callback, 
                                channels=1,
                                device=self.device_index,
                                samplerate=self.fs, 
                                blocksize=0):
                
                rospy.loginfo("マイクーストリームを開始しました。")
                rospy.spin()
                
        except Exception as e:
            rospy.logerr(f"マイク接続エラー: {e}")
            rospy.logerr("ヒント: Ubuntuの[設定] -> [サウンド] -> [入力] で Jabra が選択されているか確認してください。")
        
    def audio_callback(self, indata, frames, time_info, status):
        try:
            if status:
                print(status)

            # 初期化（未定義参照を防止）
            word = None

            data = indata[:, 0]
            vol = np.linalg.norm(data) / np.sqrt(len(data))
            if vol < 0.02:
                return

            # 2. 周波数解析 (FFT)
            window = np.hanning(len(data))
            fft_spec = np.abs(np.fft.rfft(data * window))
            fft_freq = np.fft.rfftfreq(len(data), d=1.0/self.fs)

            # 最も強い周波数を取得（あなたの設定域）
            voice_band = np.where((fft_freq >= 300) & (fft_freq <= 3000))[0]
            noise_band = np.where((fft_freq < 300) | (fft_freq > 3000))[0]

            if len(voice_band) == 0:
                return

            voice_pow = np.mean(fft_spec[voice_band])
            noise_pow = np.mean(fft_spec[noise_band]) if len(noise_band) > 0 else 0

            # 3. 判定（トリガー条件を満たせば word をセット）
            if voice_pow > self.vol_thresh * 5 and voice_pow > noise_pow:
                word = "sound_trigger_True"
                rospy.loginfo(f"voice detected! vol:{vol:.4f}, voicepow:{voice_pow:.2f}")
            else:
                # 条件を満たさなければ何もしない
                return

            # 4. デバウンス（最小発行間隔の確認）
            now = rospy.Time.now()
            if (now - getattr(self, "_last_pub_time", rospy.Time(0))).to_sec() < getattr(self, "_min_pub_interval", 0.05):
                rospy.logdebug("publish skipped by debounce")
                return

            # 最終的に publish（word が None のときは何もしない）
            if word is not None:
                self.pub_trigger.publish(word)
                self._last_pub_time = now

        except Exception as e:
            # sounddevice のコールバック内で例外が出ると不安定になるので必ず捕捉する
            rospy.logerr(f"audio_callback exception: {e}")


if __name__ == '__main__':
    SoundTriggerNode()