#!/usr/bin/env python

import roslaunch
import rospy

if __name__ == "__main__":

    rospy.init_node('batch_manager', anonymous=True)

    rate = rospy.Rate(100)

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    

    while not rospy.is_shutdown():
        naoqi_launch = roslaunch.parent.ROSLaunchParent(uuid,["/opt/ros/kinetic/share/nao_bringup/launch/nao_full_py.launch"])
        naoqi_launch.start()
        print "spinning!"
        naoqi_launch.spin()
        naoqi_launch.shutdown()
        

