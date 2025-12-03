#!/usr/bin/env python3
import rospy
import numpy as np
import sounddevice as sd
from std_msgs.msg import Time

class SoundTriggerNode:
    def __init__(self):
        rospy.init_node('sound_trigger_node', anonymous=True)
        
        # --- 設定 ---
        # この周波数以上なら反応する (Hz)
        self.target_freq = rospy.get_param('~target_freq', 2000.0)
        # 音量閾値
        self.vol_thresh = rospy.get_param('~vol_thresh', 1.0)
        
        self.pub_time = rospy.Publisher('/sound_trigger_time', Time, queue_size=10)
        
        self.fs = 44100
        self.blocksize = 0
        
        rospy.loginfo(f"監視開始:{self.target_freq}Hz以上の音を待っている")
        
        try:
            # 指定されたデバイス(5)を試す
            rospy.loginfo("デバイス5での接続を試みます...")
            dev5_info = sd.query_devices(5, 'input')
            if dev5_info['max_input_channels'] > 0:
                with sd.InputStream(callback=self.audio_callback, 
                                    channels=1,
                                    device=5,
                                    samplerate=self.fs, 
                                    blocksize=self.blocksize):
                    rospy.spin()
            else:
                rospy.logwarn(f"デバイス5は入力チャンネルを持っていません (max_input_channels=0). スキップします。")
                raise Exception("Device 5 has 0 input channels")
        except Exception as e:
            rospy.logwarn(f"デバイス5での接続に失敗しました: {e}")
            rospy.loginfo("デフォルトデバイス(default)での接続を試みます...")
            try:
                with sd.InputStream(callback=self.audio_callback, 
                                    channels=1,
                                    device='default',
                                    samplerate=self.fs, 
                                    blocksize=self.blocksize):
                    rospy.spin()
            except Exception as e2:
                rospy.logerr(f"マイクエラー (デフォルトデバイスも失敗): {e2}")
    
    def audio_callback(self, indata, frames, time, status):
        if status:
            print(status)
        
        vol = np.linalg.norm(indata) * 10
        if vol < self.vol_thresh:
            return
        
        # 2. 周波数解析 (FFT)
        data = indata[:, 0]
        window = np.hanning(len(data))
        fft_spec = np.abs(np.fft.rfft(data * window))
        fft_freq = np.fft.rfftfreq(len(data), d=1.0/self.fs)
        
        # 最も強い周波数を取得
        peak_freq = fft_freq[np.argmax(fft_spec)]
        
        # 3. 判定と時刻送信
        if peak_freq >= self.target_freq:
            # 現在のROS時刻を取得
            now = rospy.Time.now()
            
            # Publishする
            self.pub_time.publish(now)
            
            # ログ表示 (人間確認用)
            rospy.loginfo(f"検知! 時刻:{now.to_sec():.2f}, 周波数:{peak_freq:.0f}Hz")

if __name__ == '__main__':
    SoundTriggerNode()