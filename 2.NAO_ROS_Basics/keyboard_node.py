#!/usr/bin/env python
# May 6, 2020
# Nick Tacca

import rospy
from std_msgs.msg import String

def message():
    pub = rospy.Publisher('key', String, queue_size=10)
    rospy.init_node('keyboard_node', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        msg = raw_input("Enter Action (h: home, l: left wave, r: right wave, b: both wave): ")
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        message()
    except rospy.ROSInterruptException:
        pass
