#!/usr/bin/env python3

import rospy
from std_msgs.msg import String


def main():
    rospy.init_node('enter_topic_publisher')

    pub = rospy.Publisher('/enter_trigger', String, queue_size=10)

    rospy.loginfo('Enterキーを押すと /enter_trigger に "start" を publish します。')
    rospy.loginfo('終了するときは Ctrl + C を押してください。')

    while not rospy.is_shutdown():
        try:
            input()
            pub.publish("start")
            rospy.loginfo('Published: start')
        except EOFError:
            break
        except KeyboardInterrupt:
            break


if __name__ == '__main__':
    main()