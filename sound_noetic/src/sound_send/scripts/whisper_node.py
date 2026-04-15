#!/usr/bin/env python3
import os
import sys
import subprocess

import noisereduce as nr
import numpy as np
import rospy
import torch
import whisper
from std_msgs.msg import String


class WhisperNode:
    def __init__(self):
        rospy.init_node('whisper_node', anonymous=True)

        self.model_size = rospy.get_param('~model', 'medium')
        self.device_index = rospy.get_param('~device_index', '')

        # Audio processing params
        self.enable_noise_reduction = rospy.get_param('~enable_noise_reduction', True)
        self.enable_spl_filter = rospy.get_param('~enable_spl_filter', True)
        self.enable_normalize = rospy.get_param('~enable_normalize', True)
        self.spl_threshold_db = rospy.get_param('~spl_threshold_db', -42.0)
        self.frame_duration_sec = rospy.get_param('~spl_frame_duration_sec', 0.02)
        self.min_active_ratio = rospy.get_param('~spl_min_active_ratio', 0.03)

        # マイクID設定
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

        # publish/subscribe topic
        self.sub_file = rospy.Subscriber(sub_file_name, String, self.callback)
        self.pub_result = rospy.Publisher('/whisper_result', String, queue_size=10)
        self.pub_state = rospy.Publisher(pub_state_name, String, queue_size=10)

        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        rospy.loginfo(f"device: {self.device}")

        rospy.loginfo(f"whisper model '{self.model_size}'")
        try:
            self.model = whisper.load_model(self.model_size, device=self.device)
            rospy.loginfo("model load: succeeded")
            self.launch_sound_subsc()
        except Exception as e:
            rospy.logerr(f"model load: failed: {e}")
            sys.exit(1)

        rospy.spin()

    def launch_sound_subsc(self):
        rospy.loginfo("Starting sound_subsc.py after model load")
        cmd = ["rosrun", "sound_send", "sound_subsc.py", f"_mic_id:={self.mic_id}"]
        if self.device_index != "":
            cmd.append(f"_device_index:={self.device_index}")
        try:
            subprocess.Popen(cmd)
            rospy.loginfo("sound_subsc.py launched successfully")
        except Exception as e:
            rospy.logwarn(f"Failed to launch sound_subsc.py: {e}")

    def _apply_spl_filter(self, audio_data: np.ndarray, sr: int) -> np.ndarray:
        """
        音圧(dBFS)ベースで低レベルフレームを除去(0化)する。
        - フレームRMSから dBFS を計算
        - threshold 未満フレームを無音化
        - 有効フレーム比率が極端に低い場合は全体を無音扱い
        """
        if audio_data.size == 0:
            return audio_data

        frame_len = max(1, int(sr * self.frame_duration_sec))
        n_frames = int(np.ceil(len(audio_data) / frame_len))

        padded_len = n_frames * frame_len
        if padded_len > len(audio_data):
            padded = np.pad(audio_data, (0, padded_len - len(audio_data)), mode='constant')
        else:
            padded = audio_data

        frames = padded.reshape(n_frames, frame_len)
        rms = np.sqrt(np.mean(frames ** 2, axis=1) + 1e-12)
        dbfs = 20.0 * np.log10(rms + 1e-12)

        active_frame_mask = dbfs >= self.spl_threshold_db
        active_ratio = float(np.mean(active_frame_mask)) if active_frame_mask.size > 0 else 0.0

        if active_ratio < self.min_active_ratio:
            rospy.loginfo(
                f"SPL filter: active ratio too low ({active_ratio:.3f} < {self.min_active_ratio}), skip transcription"
            )
            return np.zeros_like(audio_data)

        sample_mask = np.repeat(active_frame_mask, frame_len)[:len(audio_data)]
        filtered = audio_data.copy()
        filtered[~sample_mask] = 0.0
        return filtered

    def process_audio(self,audio_data: np.ndarray,sr: int = 16000,) -> np.ndarray:
        """
        要求フロー:
          1) ノイズ処理
          2) 音圧(SPL)による低音圧除去
          3) 音量正規化
        """
        processed_audio = audio_data.astype(np.float32, copy=True)

        # 1) ノイズ処理
        if self.enable_noise_reduction:
            try:
                processed_audio = nr.reduce_noise(
                    y=processed_audio,
                    sr=sr,
                    stationary=True,
                    prop_decrease=0.8,
                    n_jobs=1,
                )
            except Exception as e:
                rospy.logwarn(f"ノイズ除去中にエラーが発生しました: {e}")
                rospy.logwarn("ノイズ除去をスキップします。")

        # 2) 音圧ベース除去
        if self.enable_spl_filter:
            processed_audio = self._apply_spl_filter(processed_audio, sr)

        # 3) 音量正規化
        if self.enable_normalize:
            max_abs = float(np.max(np.abs(processed_audio))) if processed_audio.size > 0 else 0.0
            if max_abs > 0.0:
                processed_audio = processed_audio / max_abs

        return processed_audio.astype(np.float32, copy=False)

    def callback(self, msg):
        audio_path = msg.data
        rospy.loginfo(f"audio file: {audio_path}")

        if not os.path.exists(audio_path):
            rospy.logwarn("no file")
            self.pub_state.publish("done")
            return

        try:
            audio_data = whisper.load_audio(audio_path)

            rospy.loginfo("audio process: noise reduction -> spl filter -> normalization")
            clean_audio = self.process_audio(audio_data, sr=16000)

            if np.max(np.abs(clean_audio)) < 1e-6:
                rospy.loginfo("audio skipped: below SPL threshold after processing")
                self.pub_state.publish("done")
                return

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
            rospy.logerr(f"whisper error: {e}")
            self.pub_state.publish("done")

        finally:
            # 処理が終わったら（成功・失敗に関わらず）ファイルを削除する
            if os.path.exists(audio_path):
                try:
                    os.remove(audio_path)
                    rospy.loginfo(f"Deleted file: {audio_path}")
                except Exception as e:
                    rospy.logwarn(f"Failed to delete file: {e}")


if __name__ == '__main__':
    try:
        WhisperNode()
    except rospy.ROSInterruptException:
        pass
