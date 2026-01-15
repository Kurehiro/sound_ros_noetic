#!/usr/bin/env python3
import rospy
import numpy as np
import os
from std_msgs.msg import String

class TaskListDebugNode:
    def __init__(self):
        rospy.init_node('task_list_debug_node', anonymous=True)
        
        #Subscribe topic
        self.sub_task = rospy.Subscriber('/whisper_result', String, self.task_callback)
        self.sub_please = rospy.Subscriber('/please_task', String, self.next_callback)
        #Publishe topic
        self.pub_send_task= rospy.Publisher('/send_task', String, queue_size=10)
        
        self.go_task = False
        self.all_task_list = []
        
        self.file_name = 'sentence_task.txt'
        self.file_path = '/root/HSR/catkin_ws/src/sound_ros_noetic/sound_noetic/src/sound_send/scripts/{}'.format(self.file_name)
        
        # 起動時にファイルを読み込む（ここが変更点）
        self.load_from_file()
        
        rospy.loginfo(f"TaskListDebug Node Started. Loaded tasks from: {self.file_path}")
        rospy.loginfo(f"Current Tasks: {self.all_task_list}")
        rospy.spin()
    
    def load_from_file(self):
        """起動時に既存のファイル内容をリストに読み込む"""
        if os.path.exists(self.file_path):
            try:
                with open(self.file_path, 'r', encoding='utf-8') as f:
                    lines = f.readlines()
                    # 改行文字を取り除いてリストに追加
                    self.all_task_list = [line.strip() for line in lines if line.strip()]
                rospy.loginfo(f"Loaded {len(self.all_task_list)} tasks from file.")
            except Exception as e:
                rospy.logerr(f"Failed to read file: {e}")
        else:
            rospy.logwarn(f"File not found: {self.file_path}. Starting with empty list.")

    def save_to_file(self):
        try:
            with open(self.file_path, 'w', encoding='utf-8') as f:
                for task in self.all_task_list:
                    f.write(task + '\n')
        except Exception as e:
            rospy.logerr(f"faile write error: {e}")
    
    def task_callback(self,msg):
        mic_task_text = msg.data
        #マイクIDとタスク内容をリストに保存
        self.all_task_list.append(mic_task_text)
        rospy.loginfo(f"list append {mic_task_text}")
        rospy.loginfo(f"Remaining Tasks: {len(self.all_task_list)}")
        self.save_to_file()

    def next_callback(self,msg):
        rospy.loginfo("next task publish")
        if len(self.all_task_list) > 0:
            item = self.all_task_list.pop(0)
            
            msg_out = String(data=item)
            self.pub_send_task.publish(msg_out)
            
            rospy.loginfo(f"Send Task: {item}")
            rospy.loginfo(f"Remaining Tasks: {len(self.all_task_list)}")
            
            self.save_to_file()
        else:
            rospy.loginfo("No tasks in the list.")

if __name__ == '__main__':
    try:
        TaskListDebugNode()
    except rospy.ROSInterruptException:
        pass
