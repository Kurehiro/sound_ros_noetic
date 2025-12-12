#!/usr/bin/env python3
import rospy
import socket
from std_msgs.msg import String

def start_server():
    rospy.init_node('socket_receiver')
    pub = rospy.Publisher('/whisper_result', String, queue_size=10)
    
    host = '0.0.0.0'
    port = 9080
    
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server_socket.bind((host, port))
    server_socket.listen(1)
    
    rospy.loginfo(f"socket受信サーバー起動: Port{port}")
    
    while not rospy.is_shutdown():
        try:
            conn, addr = server_socket.accept()
            with conn:
                rospy.loginfo(f"接続:{addr}")
                data = conn.recv(1024)
                if data:
                    text = data.decode('utf-8')
                    rospy.loginfo(f"受信テキスト: {text}")
                    pub.publish(text)
        except socket.error as e:
            pass

if __name__ == '__main__':
    start_server()