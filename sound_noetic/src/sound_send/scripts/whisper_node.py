#!/usr/bin/env python3
import rospy
import whisper
import torch
import os
import sys
import numpy as np
import noisereduce as nr
import subprocess

from std_msgs.msg import String

class WhisperNode:
    def __init__(self):
        rospy.init_node('whisper_node', anonymous=True)
        
        self.model_size = rospy.get_param('~model','medium')
        self.device_index = rospy.get_param('~device_index', '')
        
        #マイクID設定
        self.mic_id = rospy.get_param('~mic_id', 'default_mic')
        if self.mic_id == 'f':
            sub_file_name = '/first_mic/audio_path'
            pub_state_name = '/first_mic/whisper_state'
            self.send_mic_id = 'first'
        elif self.mic_id == 's':
            sub_file_name = '/second_mic/audio_path'
            pub_state_name = '/second_mic/whisper_state'
            self.send_mic_id = 'second'
        else:
            sub_file_name = f'/{self.mic_id}/audio_path'
            pub_state_name = f'/{self.mic_id}/whisper_state'
            self.send_mic_id = self.mic_id
        
        #publish topic
        self.sub_file = rospy.Subscriber(sub_file_name, String, self.callback)
        #subscribe topic
        self.pub_result = rospy.Publisher('/whisper_result', String, queue_size=10)
        self.pub_state = rospy.Publisher(pub_state_name, String, queue_size=10)
        
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        rospy.loginfo(f"device:{self.device}")
        
        rospy.loginfo(f"whisper model '{self.model_size}")
        try:
            self.model = whisper.load_model(self.model_size, device=self.device)
            rospy.loginfo("model load: succeeded")
            self.launch_sound_subsc()
        except Exception as e:
            rospy.logerr(f"model load: failed")
            sys.exit(1)
        
        rospy.spin()
    
    def launch_sound_subsc(self):
        rospy.loginfo("Starting sound_subc.py after model load")
        cmd = ["rosrun","sound_send","sound_subsc.py",f"_mic_id:={self.mic_id}"]
        if self.device_index != "":
            cmd.append(f"_device_index:={self.device_index}")
        try:
            subprocess.Popen(cmd)
            rospy.loginfo("sound_subsc.py launched successfuly")
        except Exception as e:
            rospy.loginfo(f"Failed to launch sound_subsc.py: {e}")
    
    def process_audio(self, audio_data: np.ndarray, sr: int = 16000, normalize: bool = True, reduce_noise: bool = True) -> np.ndarray:
        processed_audio = audio_data.copy()
        
        if normalize:
            # 音量正規化 (float32 を想定)
            max_abs = np.max(np.abs(processed_audio))
            if max_abs > 0:
                processed_audio = processed_audio / max_abs
        
        if reduce_noise:
            # ノイズ除去
            # noisereduce はサンプリングレート(sr)を必要とします
            try:
                processed_audio = nr.reduce_noise(
                    y=processed_audio, 
                    sr=sr, 
                    stationary=True,
                    prop_decrease=0.8,  # ノイズ除去の強度 (元のスクリプトから引用)
                    n_jobs=1
                )
            except Exception as e:
                rospy.warning(f"ノイズ除去中にエラーが発生しました: {e}")
                rospy.warning("ノイズ除去をスキップします。")
        
        return processed_audio
    

    def callback(self, msg):
        audio_path = msg.data
        rospy.loginfo(f"audio file: {audio_path}")
        
        if not os.path.exists(audio_path):
            rospy.logwarn("no file")
            self.pub_state.publish("done")
            return
        
        try:
            audio_data = whisper.load_audio(audio_path)
            
            rospy.loginfo("noise reduce process")
            clean_audio = self.process_audio(audio_data, sr=16000)
            
            rospy.loginfo("whisper start")
            result = self.model.transcribe(
                clean_audio,
                language="en",
                fp16=torch.cuda.is_available()
            )
            
            result_text = result["text"].strip()
            
            if result_text:
                message = f"{self.send_mic_id}:{result_text}"
                rospy.loginfo(f"result: {message}")
                self.pub_result.publish(message)

                self.pub_state.publish("done")
            else:
                rospy.loginfo("no result")
                self.pub_state.publish("done")
        
        except Exception as e:
            rospy.logerr(f"whisper error")
            self.pub_state.publish("done")

if __name__ == '__main__':
    try:
        WhisperNode()
    except rospy.ROSInterruptException:
        pass