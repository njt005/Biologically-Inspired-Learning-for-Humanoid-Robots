#!/usr/bin/env python
# May 13, 2020
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


    def central_execute(self, Wh1, b1, Wh2, b2, Wo, bo):
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
            X = np.array([self.BlobX/320, self.BlobY/240])

            # Adjust shoulder position based on input from red blob
            H1, H2, output = nn.feed_forward(X, Wh1, b1, Wh2, b2, Wo, bo)

            RShoulderPitch = 2*2.0857 * output[0] - 2.0857
            RShoulderRoll = (0.3142 + 1.3265) * output[1] - 1.3265

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

class NeuralNet:

    def __init__(self, n_inputs, h_layer1, h_layer2, n_outputs, epochs, batch_size, lr):
        self.n_inputs = n_inputs
        self.h_layer1 = h_layer1
        self.h_layer2 = h_layer2
        self.n_outputs = n_outputs
        self.epochs = epochs
        self.batch_size = batch_size
        self.lr = lr

    def activation_function(self, x, name):
        if name == "Sigmoid":
            f = 1/(1 + np.exp(-x))
        elif name == "Tanh":
            f = (np.exp(2*x) - 1) / (np.exp(2*x) + 1)
        elif name == "ReLU":
            f = x * (x > 0)
        elif name == "Leaky_ReLU":
            f = np.where(x > 0, x, x * 0.1)
        else:
            raise ValueError('Activation function not supported')
        return f
    
    def der_activation_function(self, x, name):
        if name == "Sigmoid":
            f = 1/(1 + np.exp(-x))
            df = f*(1 - f)
        elif name == "Tanh":
            f = (np.exp(2*x) - 1) / (np.exp(2*x) + 1)
            df = 1 - f*f
        elif name == "ReLU":
            df = 1. * (x > 0)
        elif name == "Leaky_ReLU":
            df = np.where(x > 0, 1, 0.01)
        else:
            raise ValueError('Activation function not supported')
        return df
    
    def softmax(self, x):
        num = np.exp(x - np.max(x))
        den = np.sum(num)
        return num / den
    
    def feed_forward(self, X, Wh1, b1, Wh2, b2, Wo, bo):
        # Hidden layer 1
        Zh1 = np.dot(Wh1, X) + b1
        H1 = self.activation_function(Zh1, "Sigmoid")

        # Hidden layer 2
        Zh2 = np.dot(Wh2, H1) + b2
        H2 = self.activation_function(Zh2, "Sigmoid")

        # Output layer
        Zo = np.dot(Wo, H2) + bo
        output = self.activation_function(Zo, "Sigmoid")
        return H1, H2, output

    def compute_loss_func(self, y_pred, y_true, name): 
        if name == "MSE":
            e = np.mean(np.power(y_true-y_pred, 2))
            # de = 2*(y_pred-y_true)/y_true.size
        elif name == "cross_entropy":
            n_samples = y_true.shape[0]
            #y_real.argmax(axis=1) returns the indices of 
            logp = -np.log(y_pred[np.arange(n_samples), y_true.argmax(axis=1)])
            #log_likelihood = -np.log(p[range(a),y_true])
            e = np.sum(logp)/n_samples
            #p[range(a),y_true] -= 1
            # de = 0
        else:
            raise ValueError('Cost function is not supported')
        return e
    
    def backpropagate(self, X, Wh1, b1, H1, Wh2, b2, H2, Wo, bo, output, y_true):

        # Derivatives
        dZo = output - y_true
        dWo = np.dot(dZo, H2.T)
        dbo = np.sum(dZo, axis=1)
        dZh2 = np.multiply(np.dot(Wo.T, dZo), self.der_activation_function(H2, "Sigmoid"))
        dWh2 = np.dot(dZh2, H1.T)
        db2 = np.sum(dZh2, axis=1)
        dZh1 = np.multiply(np.dot(Wh2.T, dZh2), self.der_activation_function(H1, "Sigmoid"))
        dWh1 = np.dot(dZh1, X.T)
        db1 = np.sum(dZh1, axis=1)
        
        # Updates
        Wh1 = Wh1 - self.lr * dWh1
        b1 = b1 - self.lr * db1
        Wh2 = Wh2 - self.lr * dWh2
        b2 = b2 - self.lr * db2
        Wo = Wo - self.lr * dWo
        bo = bo - self.lr * dbo

        return Wh1, b1, Wh2, b2, Wo, bo

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
    data = np.loadtxt(os.path.join(dirname, 'training_data3.csv'), delimiter=',')
    cx = data[:,0]
    cy = data[:,1]
    RShoulderPitch = data[:,2]
    RShoulderRoll = data[:,3]
    
    # Normalize training data between 0 & 1
    cx /= 320
    cy /= 240
    RShoulderPitch = (RShoulderPitch + 2.0857) / (2*2.0857)
    RShoulderRoll = (RShoulderRoll + 1.3265) / (0.3142 + 1.3265)

    # Reshape input and output data
    X_train = np.vstack((cx, cy))
    X_train = X_train.reshape(2, len(cx))
    y_train = np.vstack((RShoulderPitch, RShoulderRoll))
    y_train = y_train.reshape(2, len(RShoulderPitch))

    # Network structure
    n_inputs = X_train.shape[0]
    h_layer1 = 300
    h_layer2 = 300
    n_outputs = y_train.shape[0]

    train = yes_or_no("Would you like to train a new network?")

    #Hyperparameters
    epochs = 200
    lr = 0.1
    batch_size = 4

    # Instantiate neural net class
    nn = NeuralNet(n_inputs, h_layer1, h_layer2, n_outputs, epochs, batch_size, lr)

    if train:

        # Initialize weights -- bias = 0
        Wh1 = np.random.normal(-0.1, 0.1, (h_layer1, n_inputs))
        b1 = np.zeros(h_layer1)
        Wh2 = np.random.normal(-0.1, 0.1, (h_layer2, h_layer1))
        b2 = np.zeros(h_layer2)
        Wo = np.random.normal(-0.1, 0.1, (n_outputs, h_layer2))
        bo = np.zeros(n_outputs)

        # Number of training samples & batches
        m = X_train.shape[1]
        num_batches = int(m / batch_size)

        # Main training loop
        total_loss = []
        print("\nTraining...\n")

        for epoch in range(0, epochs):
            
            # Shuffle training data each epoch
            np.random.seed(138)
            shuffle_index = np.random.permutation(m)
            X_train, y_train = X_train[:,shuffle_index], y_train[:,shuffle_index]

            # Reshape data and labels for batches
            X_train = X_train.reshape((n_inputs, num_batches, batch_size))
            y_train = y_train.reshape((n_outputs, num_batches, batch_size))

            epoch_loss = []

            for iteration in range(0, num_batches):

                # Initialize training vectors
                H1 = np.zeros((h_layer1, batch_size))
                H2 = np.zeros((h_layer2, batch_size))
                output = np.zeros((n_outputs, batch_size))

                for i in range(0, batch_size):
                    H1[:,i], H2[:,i], output[:,i] = nn.feed_forward(X_train[:,iteration,i], Wh1, b1, Wh2, b2, Wo, bo)

                    # # Expand output from -1 to 1 to each range (RShoulderPitch, RShoulderRoll) --- this was before I normalized input and output Tanh
                    # output = np.vstack((output[0,:] * 2.0857, (0.3142 + 1.3265) * (output[1,:] + 1) / 2 -1.3265))

                # Compute error for batch and backpropagate
                loss = nn.compute_loss_func(output, y_train[:,iteration,:], "MSE")
                Wh1, b1, Wh2, b2, Wo, bo = nn.backpropagate(X_train[:,iteration,:], Wh1, b1, H1, Wh2, b2, H2, Wo, bo, output, y_train[:,iteration,:])

                if int(iteration+1) % 1 == 0:
                    print("Epoch: {}/{}, Iteration: {}/{}, Loss: {:.2f}".format(epoch+1,epochs,iteration+1,num_batches,loss))

                # Append epoch loss and epoch accuracy vectors
                epoch_loss.append(loss)

            # Reshape data for shuffling next epoch
            X_train = X_train.reshape((n_inputs, m))
            y_train = y_train.reshape((n_outputs, m))

            # Make arrays
            epoch_loss = np.vstack(epoch_loss)

            # Append total loss and total accuracy vectors
            total_loss.append(epoch_loss)
        
        # Make arrays
        total_loss = np.vstack(total_loss)

        # Plotting loss curve
        plt.plot(total_loss)
        plt.xlabel('Training Episodes')
        plt.ylabel('Mean Squared Error')
        plt.title('Learning Curve')
        plt.grid()
        plt.show()

        # Saving Network Params
        network_params = [Wh1, b1, Wh2, b2, Wo, bo]
        network_param_names = ["Wh1", "b1", "Wh2", "b2", "Wo", "bo"]
        for i,j in zip(network_params, network_param_names):
            np.savetxt(os.path.join(dirname, "network_params/{}.dat".format(j)), i)
    
    else:

        # Load network paramsn
        network_param_names = ["Wh1", "b1", "Wh2", "b2", "Wo", "bo"]
        Wh1 = np.loadtxt(os.path.join(dirname, "network_params/Wh1.dat"))
        b1 = np.loadtxt(os.path.join(dirname, "network_params/b1.dat"))
        Wh2 = np.loadtxt(os.path.join(dirname, "network_params/Wh2.dat"))
        b2 = np.loadtxt(os.path.join(dirname, "network_params/b2.dat"))
        Wo = np.loadtxt(os.path.join(dirname, "network_params/Wo.dat"))
        bo = np.loadtxt(os.path.join(dirname, "network_params/bo.dat"))
    
    return nn, Wh1, b1, Wh2, b2, Wo, bo


if __name__=='__main__':
    
    # Run neural net first
    nn, Wh1, b1, Wh2, b2, Wo, bo = main()

    # Instantiate class and start loop function
    central_instance = Central(nn)
    central_instance.central_execute(Wh1, b1, Wh2, b2, Wo, bo)
