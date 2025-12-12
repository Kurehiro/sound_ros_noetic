#!/usr/bin/env python3
import rospy
import openai
from std_msgs.msg import String

class JudgeTaskNode:
    def __init__(self):
        rospy.init_node('judge_task_node', anonymous=True)
        
        self.api_path = "/root/HSR/catkin_ws/src/sound_ros_noetic/sound_noetic/src/sound_send/scripts/GPT_API_Key.txt"
        self.prompt_path = "/root/HSR/catkin_ws/src/sound_ros_noetic/sound_noetic/src/sound_send/scripts/prompt_judege.txt"
        
        self.sub_task_list = rospy.Subscriber('/send_task', String, self.GPT_callback)
        
        self.pub_GPT_result = rospy.Publisher('/GPT_result', String, queue_size=10)
        
        self.task_result = []
        try:
            with open(self.prompt_path, 'r') as f:
                self.prompt_content = f.read()
            with open(self.api_path, 'r') as f:
                self.api_key = f.read().strip()
        except Exception as e:
            rospy.logerr(f"faile erorr: {e}")
            return
        
        rospy.loginfo("JudgeTaskNode Ready: waiting for /send_task")
        rospy.spin()
    
    def GPT_prompt(self,task_text):
        """GPTによるテキスト解析によるタスク判別関数"""
        try:
            prompt = [{"role":"system", "content":self.prompt_content}]
            input_task = [{"role":"user", "content":task_text}]
            
            message = prompt + input_task
            
            response = openai.chat.completions.create(
                model = "gpt-4o",
                messages = message,
                temperature = 0.0,
                max_tokens = 2048
            )
            
            context = response.choices[0].message.content
            
            return context
        except Exception as e:
            rospy.logerr(f"GPT API Error: {e}")
    
    def GPT_callback(self,msg):
        task = msg.data
        rospy.loginfo(f"Analyzing: {task}")
        
        result = self.GPT_prompt(task)
        
        msg_out = String(data=result)
        self.pub_GPT_result.publishe(msg_out)
        
        rospy.loginfo(f"Result Published: {result}")

if __name__ == "__main__":
    try:
        JudgeTaskNode()
    except rospy.ROSInterruptException:
        pass