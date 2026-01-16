#!/usr/bin/env python3
import rospy
import openai
from std_msgs.msg import String

class JudgeTaskNode:
    def __init__(self):
        rospy.init_node('judge_task_node', anonymous=True)

        self.result_task = "/root/HSR/catkin_ws/src/sound_ros_noetic/sound_noetic/src/sound_send/scripts/task_list.txt"
        self.api_path = "/root/HSR/catkin_ws/src/sound_ros_noetic/sound_noetic/src/sound_send/sentence_text/GPT_API_Key.txt"
        self.prompt_path = "/root/HSR/catkin_ws/src/sound_ros_noetic/sound_noetic/src/sound_send/sentence_text/prompt_judge.txt"
        
        self.sub_task_list = rospy.Subscriber('/send_task', String, self.GPT_callback)
        
        self.pub_GPT_result = rospy.Publisher('/GPT_result', String, queue_size=10)
        
        self.task_result = []
        try:
            with open(self.prompt_path, 'r') as f:
                self.prompt_content = f.read()
            with open(self.api_path, 'r') as f:
                openai.api_key = f.read().strip()
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
                model="gpt-5-mini",
                messages=message,
                temperature=1.0,
                max_completion_tokens=2048
            )
            
            context = response.choices[0].message.content
            return context
        
        except Exception as e:
            rospy.logerr(f"GPT API Error: {e}")
            return "no result"
    
    def GPT_callback(self,msg):
        task = msg.data
        rospy.loginfo(f"Analyzing: {task}")
        
        result = self.GPT_prompt(task)
        
        rospy.loginfo(f"GPT response: {result}")
        msg_out = String(data=result)
        self.task_result.append(result)
        self.pub_GPT_result.publish(msg_out)
        
        try:
            with open(self.result_task, 'w', encoding='utf-8') as f:
                for t in self.task_result:
                    f.write(t + '\n')
            rospy.loginfo(f"Task Added to List: {result}")
        except Exception as e:
            rospy.logerr(f"File Write Error: {e}")

        rospy.loginfo(f"Result Published: {result}")

if __name__ == "__main__":
    try:
        JudgeTaskNode()
    except rospy.ROSInterruptException:
        pass