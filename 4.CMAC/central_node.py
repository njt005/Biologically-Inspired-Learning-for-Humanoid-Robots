#!/usr/bin/env python
# May 20, 2020
# Nick Tacca

from __future__ import division
import rospy
import numpy as np
import matplotlib.pyplot as plt
from std_msgs.msg import String
from std_srvs.srv import Empty
from naoqi_bridge_msgs.msg import JointAnglesWithSpeed,Bumper,HeadTouch
from sensor_msgs.msg import Image,JointState
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os

class Central:

    def __init__(self, nn):
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

        # Find contours in the binary image
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

    def set_joint_angles(self,HeadYaw_angle, LShoulderPitch_angle, RElbowRoll_angle, RShoulderPitch_angle, RShoulderRoll_angle):

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
        joint_angles_to_set.relative = False # if true you can increment positions
        joint_angles_to_set.speed = 0.1 # keep this low if you can
        self.jointPub.publish(joint_angles_to_set)


    def central_execute(self, W3, a):
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

        while True:

            rospy.sleep(1.0)

            # Get centroid of blob position to feed into network
            X = np.array([np.around(self.BlobX/320, 1), np.around(self.BlobY/240, 1)])

            # Adjust shoulder position based on input from red blob
            L2, output = cb.cerebellum(X, W3, a)
            
            RShoulderPitch = np.around(2*2.0857 * output[0] - 2.0857, 1)
            RShoulderRoll = np.around((0.3142 + 1.3265) * output[1] - 1.3265, 1)

            # print("cx: {:.2f}, cy: {:.2f}".format(self.BlobX, self.BlobY))
            # print("RShoulderPitch: {:.2f}, RShoulderRoll: {:.2f}".format(RShoulderPitch, RShoulderRoll))

            self.set_joint_angles(0, 1.5, 1, RShoulderPitch, RShoulderRoll)

        self.set_stiffness(False) # always check that your robot is in a stable position before disabling the stiffness!!

        rospy.spin()

        while not rospy.is_shutdown():
            self.set_stiffness(self.stiffness)
            rate.sleep()

    # each Subscriber is handled in its own thread
    #rospy.spin()

class CMAC:

    def __init__(self, n_inputs, res, na, n_outputs, epochs, lr):
        self.n_inputs = n_inputs
        self.res = res
        self.na = na
        self.n_outputs = n_outputs
        self.epochs = epochs
        self.lr = lr
    
    def layer2(self):
        a = np.zeros((self.res, self.res))
        arr1 = np.random.permutation(self.na)
        arr2 = np.random.permutation(self.na)
        for z1 in range(0, self.res):
            for z2 in range(0, self.res):
                for i in range(0, self.na):
                    if ((z1%self.na) == arr1[i]) and ((z2%self.na) == arr2[i]):
                        a[z1][z2] = 1
        return a

    
    def cerebellum(self, X, W3, a):
        
        # Layer 1
        L1 = np.zeros((self.res, self.n_inputs))
        idx = X*self.res-1
        for i in range(len(X)):
            L1[idx[i]-int(self.na/2):idx[i]+int(self.na/2),i] = 1
        
        # Layer 2
        L2 = np.zeros((self.res,self.res))
        for i in range(0, self.res):
            for j in range(0, self.res):
                # If Layer 2 neuron is there...
                if a[i,j] == 1:
                    # ...and all inputs are nonzero
                    if L1[i,0] == L1.T[1,j] == 1:
                        L2[i,j] = 1

        # Layer 3
        L3 = np.zeros(self.n_outputs)
        for i in range(self.n_outputs):
            L3[i] = np.sum(np.multiply(W3[i,:,:],L2))

        return L2, L3
    
    def MSE(self, y_pred, y_true):
        return np.mean(np.power(y_true-y_pred, 2))
    
    def update_weights(self, W3, L2, y_pred, y_true):
        for n in range(len(y_pred)):
            W3[n,:,:] += self.lr * L2 / self.na * (y_true[n] - y_pred[n])
        return W3

def yes_or_no(question):
    while "The answer is invalid":
        reply = str(raw_input(question+' (y/n): ')).lower().strip()
        if reply[:1] == 'y':
            return True
        if reply[:1] == 'n':
            return False
    
def main():

    # Load training data
    dirname = os.path.dirname(os.path.abspath(__file__))

    # Automated training samples
    data = np.loadtxt(os.path.join(dirname, 'training_data_800.dat'))
    data = data.reshape(4,800).T

    # # Manual training samples
    # data = np.loadtxt(os.path.join(dirname, 'training_data_150.csv'), delimiter=',')
    # data = np.loadtxt(os.path.join(dirname, 'training_data_75.csv'), delimiter=',')

    # Assign data to arrays
    cx = data[:,0]
    cy = data[:,1]
    RShoulderPitch = data[:,2]
    RShoulderRoll = data[:,3]

    # Normalize centroid training data between 0 & 1
    cx = np.around(cx/320, 1)
    cy = np.around(cy/240, 1)

    # Normalize joint angle training data between 0 & 1
    RShoulderPitch = np.around((RShoulderPitch + 2.0857) / (2*2.0857), 5)
    RShoulderRoll = np.around((RShoulderRoll + 1.3265) / (0.3142 + 1.3265), 5)

    # Reshape input and output data
    X_train = np.vstack((cx, cy))
    X_train = X_train.reshape(2, len(cx))
    y_train = np.vstack((RShoulderPitch, RShoulderRoll))
    y_train = y_train.reshape(2, len(RShoulderPitch))

    # CMAC structure
    n_inputs = X_train.shape[0]
    res = 50
    na = 5
    n_outputs = y_train.shape[0]

    train = yes_or_no("Would you like to train a CMAC model?")

    #Hyperparameters
    epochs = 20
    lr = 0.6

    # Instantiate neural net class
    cb = CMAC(n_inputs, res, na, n_outputs, epochs, lr)

    if train:

        # Initialize weights
        a = cb.layer2()
        W3 = np.random.normal(0, 0.1, (n_outputs, res, res))
        for n in range(n_outputs):
            W3[n] = np.multiply(W3[n], a)

        # Number of training samples & batches
        m = X_train.shape[1]

        # Main training loop
        total_loss = []
        print("\nTraining...\n")

        for epoch in range(0, epochs):
            
            # Shuffle training data each epoch
            np.random.seed(138)
            shuffle_index = np.random.permutation(m)
            X_train, y_train = X_train[:,shuffle_index], y_train[:,shuffle_index]

            # Initialize output vector
            output = np.zeros((n_outputs, m))
            loss = 0
            for i in range(0, m):
                L2, output[:,i] = cb.cerebellum(X_train[:,i], W3, a)

                # Compute error and update weights
                loss += cb.MSE(output[:,i], y_train[:,i])
                W3 = cb.update_weights(W3, L2, output[:,i], y_train[:,i])

            print("Epoch: {}/{}, Loss: {:.3f}".format(epoch+1,epochs,loss))

            # Append epoch loss and epoch accuracy vectors
            total_loss.append(loss)
        
        # Make arrays
        total_loss = np.vstack(total_loss)

        # Plotting loss curve
        plt.plot(total_loss)
        plt.xlabel('Training Epochs')
        plt.ylabel('Mean Squared Error')
        plt.title('Learning Curve')
        plt.grid()
        plt.savefig(os.path.join(dirname, "Training_Nsamps-{}_RF-{}.png".format(m, na)), dpi=500)
        plt.show()

        # Saving CMAC weights
        W3 = W3.reshape((n_outputs, res*res))
        np.savetxt(os.path.join(dirname, "CMAC_weights/W3.dat"), W3)
        np.savetxt(os.path.join(dirname, "CMAC_weights/a.dat"), a)
        W3 = W3.reshape((n_outputs, res, res))
    
    else:

        # Load CMAC weights
        W3 = np.loadtxt(os.path.join(dirname, "CMAC_weights/W3.dat"))
        a = np.loadtxt(os.path.join(dirname, "CMAC_weights/a.dat"))
        W3 = W3.reshape((n_outputs, res, res))
    
    return cb, W3, a


if __name__=='__main__':
    
    # Run CMAC first
    cb, W3, a = main()

    # Instantiate class and start loop function
    central_instance = Central(cb)
    central_instance.central_execute(W3, a)
