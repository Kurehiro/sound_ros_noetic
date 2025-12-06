#!/usr/bin/env python3
import rospy
import whisper
import torch
import os
import sys
import numpy as np
import noisereduce as nr
import socket
from std_msgs.msg import String

class WhisperNode:
    def __init__(self):
        rospy.init_node('whisper_node', anonymous=True)
        
        self.robot_ip = rospy.get_param('~robot_ip', '172.17.0.1')
        self.robot_port = rospy.get_param('~robot_port', 9080)
        
        
        self.model_size = rospy.get_param('~model','medium')
        self.sub_file = rospy.Subscriber('/audio_path', String, self.callback)
        self.pub_result = rospy.Publisher('/whisper_result', String, queue_size=10)
        self.pub_state = rospy.Publisher('/whisper_state', String, queue_size=10)
        
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        rospy.loginfo(f"device:{self.device}")
        
        rospy.loginfo(f"whisper model '{self.model_size}")
        try:
            self.model = whisper.load_model(self.model_size, device=self.device)
            rospy.loginfo("model load: succeeded")
        except Exception as e:
            rospy.logerr(f"model load: failed")
            sys.exit(1)
        
        rospy.spin()
    
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
    
    def send_to_robot(self,text):
        try:
            rospy.loginfo(f"SocketSend -> {self.robot_ip}:{self.robot_port}")
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.settimeout(2.0)
                s.connect((self.robot_ip, self.robot_port))
                s.sendall(text.encode('utf-8'))
            rospy.loginfo(f'SocketSend: succeeded')
        except Exception as e:
            rospy.logerr(f"SocketSend: failed:{e}")
    
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
                rospy.loginfo(f"result: {result_text}")
                self.pub_result.publish(result_text)
                self.send_to_robot(result_text)                
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