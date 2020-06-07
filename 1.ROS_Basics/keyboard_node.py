#!/usr/bin/env python

import rospy
from std_msgs.msg import String

def message():
    pub = rospy.Publisher('chatter', String, queue_size=10)
    rospy.init_node('keyboard_node', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        # hello_str = "hello world %s" % rospy.get_time()
        msg = raw_input("Type a message: ")
        # rospy.loginfo(hello_str)
        # pub.publish(hello_str)
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        message()
    except rospy.ROSInterruptException:
        pass