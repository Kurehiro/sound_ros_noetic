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
        
        self.pub_trigger = rospy.Publisher('/sound_trigger', String, queue_size=10)
        
        self.fs = rospy.get_param('~sample_rate', 48000)
        
        # デバイスIDを指定したい場合はlaunchファイル等で設定可能にする
        # 指定がなければ None (= システムのデフォルトマイクを使用)
        self.device_index = rospy.get_param('~device_index', None) 
        
        rospy.loginfo(f"監視開始: {self.target_freq}Hz以上の音を待っています")
        
        # デバイス情報のログ表示
        if self.device_index is None:
            rospy.loginfo("使用デバイス: システムデフォルト (Ubuntuの設定に従います)")
        else:
            rospy.loginfo(f"使用デバイスID: {self.device_index}")
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
        
    def audio_callback(self, indata, frames, time, status):
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
        
        # 最も強い周波数を取得
        voice_band = np.where((fft_freq >= 300) & (fft_freq <= 3000))[0]
        noise_band = np.where((fft_freq < 300) | (fft_freq > 3000))[0]
        
        if len(voice_band) == 0:
            return
        
        voice_pow = np.mean(fft_spec[voice_band])
        noise_pow = np.mean(fft_spec[noise_band]) if len(noise_band) > 0 else 0
        
        # 3. 判定と時刻送信
        if voice_pow > self.vol_thresh * 5 and voice_pow > noise_pow:
            # 現在のROS時刻を取得
            word = "sound_trigger_True"
            
            # Publishする
            self.pub_trigger.publish(word)
            
            # ログ表示
            rospy.loginfo(f"voice detected! vol:{vol:.4f}, voicepow:{voice_pow:.2f}")

if __name__ == '__main__':
    SoundTriggerNode()