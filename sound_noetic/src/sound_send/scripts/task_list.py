import rospy
import numpy as np
from std_msgs.msg import String

class TaskList:
    def __init__(self):
        rospy.init_node('tas_list', anonymous=True)
        
        self.sub_task = rospy.Subscriber('/whisper_result', String, self.task_callback)
        self.sub_please = rospy.Subscriber('/please_task', String, self.next_callback)
        
        self.pub_send_task= rospy.Publisher('/send_task', String, queue_size=10)
        
        self.go_task = False
        self.all_task_list = []
        
    def task_callback(self,msg):
        mic_task_text = msg.data
        #マイクIDとタスク内容をリストに保存
        self.all_task_list.append(mic_task_text)
        rospy.loginfo(f"list append {mic_task_text}")
        rospy.loginfo(f"Remaining Tasks: {len(self.all_task_list)}")

    def next_callback(self,msg):
        rospy.loginfo("next task publish")
        if len(self.all_task_list) > 0:
            item = self.all_task_list.pop(0)
            
            self.pub_send_task.publish(item)
            
            rospy.loginfo(f"Send Task: {self.pub_send_task}")
            rospy.loginfo(f"Remaining Tasks: {len(self.all_task_list)}")
        else:
            rospy.loginfo("No tasks in the list.")

if __name__ == '__main__':
    try:
        TaskList()
    except rospy.ROSInterruptException:
        pass