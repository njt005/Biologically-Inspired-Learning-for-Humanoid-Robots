#!/usr/bin/env python
# May 20, 2020
# Nick Tacca

import rospy
import numpy as np
from std_msgs.msg import String
from std_srvs.srv import Empty
from naoqi_bridge_msgs.msg import JointAnglesWithSpeed,Bumper,HeadTouch
from sensor_msgs.msg import Image,JointState
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os

class Central:


    def __init__(self):
        # initialize class variables
        self.joint_names = []
        self.joint_angles = []
        self.joint_velocities = []
        self.jointPub = 0
        self.stiffness = False  
        self.key = ""
        self.BlobX = 0
        self.BlobY = 0

        pass


    def key_cb(self,data):
        # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
        self.key = data.data

    def joints_cb(self,data):
        #rospy.loginfo("joint states "+str(data.name)+str(data.position))
        # store current joint information in class variables
        self.joint_names = data.name 
        self.joint_angles = data.position
        self.joint_velocities = data.velocity

        pass

    def bumper_cb(self,data):
        rospy.loginfo("bumper: "+str(data.bumper)+" state: "+str(data.state))
        if data.bumper == 0:
            self.stiffness = True
        elif data.bumper == 1:
            self.stiffness = False

    def touch_cb(self,data):
        rospy.loginfo("touch button: "+str(data.button)+" state: "+str(data.state))

    def image_cb(self,data):
        bridge_instance = CvBridge()
        try:
            cv_image = bridge_instance.imgmsg_to_cv2(data,"bgr8")
        except CvBridgeError as e:
            rospy.logerr(e)
        
        # Red detection -- 2 masks required
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        lower_range1 = np.array([0,140,0])
        upper_range1 = np.array([15,255,255])
        mask1 = cv2.inRange(hsv, lower_range1, upper_range1)

        lower_range2 = np.array([170,140,0])
        upper_range2 = np.array([185,255,255])
        mask2 = cv2.inRange(hsv, lower_range2, upper_range2)

        mask = mask1 + mask2

        # Convert mask to binary image
        ret,thresh = cv2.threshold(mask,127,255,0)

        # Find countours in the binary image
        im2, contours, hierarchy = cv2.findContours(thresh,cv2.RETR_TREE,cv2.CHAIN_APPROX_SIMPLE)
        areas = []
        for c in contours:
            area = cv2.contourArea(c)
            areas.append(area)

        # Index of largest blob
        if len(areas) != 0:
            idx = np.argmax(areas)     

            # Get moment of largest blob   
            M = cv2.moments(contours[idx])

            # Calculate x,y coordinate of centroid
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])

            #print("Centroid of largest blob: {}, {}".format(cx, cy))

            # Show centroid in image
            cv2.circle(cv_image, (cx,cy), 5, (255,255,255), -1)

            self.BlobX = cx
            self.BlobY = cy
        
        cv2.imshow("image window",cv_image)
        cv2.imshow("mask window",mask)
        cv2.waitKey(3) # a small wait time is needed for the image to be displayed correctly

    # sets the stiffness for all joints. can be refined to only toggle single joints, set values between [0,1] etc
    def set_stiffness(self,value):
        if value == True:
            service_name = '/body_stiffness/enable'
        elif value == False:
            service_name = '/body_stiffness/disable'
        try:
            stiffness_service = rospy.ServiceProxy(service_name,Empty)
            stiffness_service()
        except rospy.ServiceException, e:
            rospy.logerr(e)

    def set_joint_angles(self,HeadYaw_angle, LShoulderPitch_angle, RElbowRoll_angle, RShoulderPitch_angle, RShoulderRoll_angle, RWristYaw_angle, RElbowYaw_angle, RHand):

        joint_angles_to_set = JointAnglesWithSpeed()
        joint_angles_to_set.joint_names.append("HeadYaw") # each joint has a specific name, look into the joint_state topic or google
        joint_angles_to_set.joint_angles.append(HeadYaw_angle) # the joint values have to be in the same order as the names!!
        joint_angles_to_set.joint_names.append("LShoulderPitch")
        joint_angles_to_set.joint_angles.append(LShoulderPitch_angle)
        joint_angles_to_set.joint_names.append("RElbowRoll")
        joint_angles_to_set.joint_angles.append(RElbowRoll_angle)
        joint_angles_to_set.joint_names.append("RShoulderPitch")
        joint_angles_to_set.joint_angles.append(RShoulderPitch_angle)
        joint_angles_to_set.joint_names.append("RShoulderRoll")
        joint_angles_to_set.joint_angles.append(RShoulderRoll_angle)
        joint_angles_to_set.joint_names.append("RWristYaw")
        joint_angles_to_set.joint_angles.append(RWristYaw_angle)
        joint_angles_to_set.joint_names.append("RElbowYaw")
        joint_angles_to_set.joint_angles.append(RElbowYaw_angle)
        joint_angles_to_set.joint_names.append("RHand")
        joint_angles_to_set.joint_angles.append(RHand)
        joint_angles_to_set.relative = False # if true you can increment positions
        joint_angles_to_set.speed = 0.1 # keep this low if you can
        self.jointPub.publish(joint_angles_to_set)


    def central_execute(self):
        rospy.init_node('central_node',anonymous=True) #initilizes node, sets name

        # create several topic subscribers
        rospy.Subscriber("key", String, self.key_cb)
        rospy.Subscriber("joint_states",JointState,self.joints_cb)
        rospy.Subscriber("bumper",Bumper,self.bumper_cb)
        rospy.Subscriber("tactile_touch",HeadTouch,self.touch_cb)
        rospy.Subscriber("/nao_robot/camera/top/camera/image_raw",Image,self.image_cb)
        self.jointPub = rospy.Publisher("joint_angles",JointAnglesWithSpeed,queue_size=10)

        # Go to home position automatically
        self.set_stiffness(True)
        rospy.sleep(1.0)

        rate = rospy.Rate(10) # sets the sleep time to 10ms

        # Training Data
        r_shoulder_pitch = np.linspace(-0.7, -0.1, 20)
        r_shoulder_pitch_flip = np.linspace(0, -0.7, 20)
        r_shoulder_pitch_arr = np.concatenate((r_shoulder_pitch, r_shoulder_pitch_flip))
        RShoulderPitch = np.tile(r_shoulder_pitch_arr, 20)
        noise = np.random.uniform(-0.01, 0.01, 800)
        RShoulderPitch += noise

        r_shoulder_roll = np.linspace(-0.6, 0.3, 40)
        RShoulderRoll = np.repeat(r_shoulder_roll, 20)
        noise = np.random.uniform(-0.01, 0.01, 800)
        RShoulderRoll += noise


        self.set_joint_angles(0, 1.5, 1, RShoulderPitch[0], RShoulderRoll[0], 0, 0, 0)
        rospy.sleep(2.0)


        if (len(RShoulderPitch) != len(RShoulderRoll)):
            print("Arrays for Shoulder Pitch and Roll do not have the same size!")
            print(len(RShoulderPitch))
            print(len(RShoulderRoll))
        else:
            print("RShoulderPitch, RShoulderRoll, BlobX, BlobY, Validity")

            cx_list = []
            cy_list = []

            # for-loop over all training data
            for i in range(len(RShoulderPitch)):
                # move shoulder joints into given position, HeadYaw = 0, RElbowRoll = 0.5
                self.set_joint_angles(0, 1.5, 1, RShoulderPitch[i], RShoulderRoll[i], 0, 0, 0)
                rospy.sleep(2.0)
                # write out shoulder joint positions and middle of detected blob
                print(self.BlobX, self.BlobY, "{:.3F}".format(RShoulderPitch[i]), "{:.3F}".format(RShoulderRoll[i]))
                cx_list.append(self.BlobX)
                cy_list.append(self.BlobY)
            
            cx_arr = np.array(cx_list)
            cy_arr = np.array(cy_list)

            # Saving data
            training_data = np.concatenate((cx_arr, cy_arr, RShoulderPitch, RShoulderRoll))
            dirname = os.path.dirname(os.path.abspath(__file__))
            np.savetxt(os.path.join(dirname, "training_data_{}.dat".format(len(RShoulderPitch))), training_data)

            
        self.set_stiffness(False) # always check that your robot is in a stable position before disabling the stiffness!!

        rospy.spin()

        while not rospy.is_shutdown():
            self.set_stiffness(self.stiffness)
            rate.sleep()

    # each Subscriber is handled in its own thread
    #rospy.spin()

if __name__=='__main__':
    # instantiate class and start loop function
    central_instance = Central()
    central_instance.central_execute()
