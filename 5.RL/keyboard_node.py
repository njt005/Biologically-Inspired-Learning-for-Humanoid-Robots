#!/usr/bin/env python
# July 9, 2020
# Nick Tacca

import rospy
from std_msgs.msg import String

def message():
    pub = rospy.Publisher('key', String, queue_size=10)
    rospy.init_node('keyboard_node', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        msg = raw_input("Enter one of the following:\n\
                        G for goal\n\
                        M for miss\n\
                        F for fall\n\
                        C for continue ")
        rospy.loginfo(msg)
        pub.publish(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        message()
    except rospy.ROSInterruptException:
        pass