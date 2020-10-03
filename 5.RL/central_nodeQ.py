#!/usr/bin/env python
# Tutorial 4 / May 20, 2020
# Franzi Hacket, Patrick Hinz, Nick Tacca

from __future__ import division
import rospy
import numpy as np
import matplotlib.pyplot as plt
import pickle
from sklearn import tree
from std_msgs.msg import String
from std_srvs.srv import Empty
from naoqi_bridge_msgs.msg import JointAnglesWithSpeed,Bumper,HeadTouch
from sensor_msgs.msg import Image,JointState
from cv_bridge import CvBridge, CvBridgeError
import cv2
import os

# Initial constants
NS_LEG = 10
NS_GK = 20
N_ACTIONS = NS_LEG
NUM_EPISODES = 200

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
        
        # cv2.imshow("image window",cv_image)
        # cv2.imshow("mask window",mask)
        # cv2.waitKey(3) # a small wait time is needed for the image to be displayed correctly

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

    def set_joint_angles(self, LHipRoll_angle, LAnkleRoll_angle, RKneePitch_angle, RHipRoll_angle, RHipPitch_angle, RAnklePitch_angle):

        joint_angles_to_set = JointAnglesWithSpeed()
        joint_angles_to_set.joint_names.append("LHipRoll")
        joint_angles_to_set.joint_angles.append(LHipRoll_angle)
        joint_angles_to_set.joint_names.append("LAnkleRoll")
        joint_angles_to_set.joint_angles.append(LAnkleRoll_angle)
        joint_angles_to_set.joint_names.append("RKneePitch")
        joint_angles_to_set.joint_angles.append(RKneePitch_angle)
        joint_angles_to_set.joint_names.append("RHipRoll")
        joint_angles_to_set.joint_angles.append(RHipRoll_angle)
        joint_angles_to_set.joint_names.append("RHipPitch")
        joint_angles_to_set.joint_angles.append(RHipPitch_angle)
        joint_angles_to_set.joint_names.append("RAnklePitch")
        joint_angles_to_set.joint_angles.append(RAnklePitch_angle)
        joint_angles_to_set.relative = False # if true you can increment positions
        joint_angles_to_set.speed = 0.8 # keep this low if you can
        self.jointPub.publish(joint_angles_to_set)
    
    def determine_state(self, env):
        # Get leg state -- 10 possible states
        min_state = env.RHipRoll_actions[NS_LEG-1]
        max_state = env.RHipRoll_actions[0]
        s1 = round((self.joint_angles[15] - min_state)/(max_state - min_state) * (NS_LEG-1))

        # Get goalie position (only need to use x-position) -- 10 possible states
        s2 = round(self.BlobX/320 * (NS_GK-1))

        return np.array((s1, s2))

    def central_execute(self, nao, env):
        rospy.init_node('central_node',anonymous=True) #initilizes node, sets name

        # create several topic subscribers
        rospy.Subscriber("key", String, self.key_cb)
        rospy.Subscriber("joint_states",JointState,self.joints_cb)
        rospy.Subscriber("bumper",Bumper,self.bumper_cb)
        rospy.Subscriber("tactile_touch",HeadTouch,self.touch_cb)
        rospy.Subscriber("/nao_robot/camera/top/camera/image_raw",Image,self.image_cb)
        self.jointPub = rospy.Publisher("joint_angles",JointAnglesWithSpeed,queue_size=10)

        self.set_stiffness(True)
        rospy.sleep(1.0)

        rate = rospy.Rate(10) # sets the sleep time to 10ms

        train = yes_or_no("Would you like to train a new agent?")

        if train:
            while True:

                # # Finding home position (balance on one leg)
                # while True:
                #     if self.key == "H1":
                #         break

                #     # Stand on one leg
                #     self.set_joint_angles(-0.379, 0.379, 0, -0.7, 0, 0)
                #     rospy.sleep(0.1)
                
                # while True:
                #     if self.key == "H2":
                #         break

                #     self.set_joint_angles(-0.379, 0.379, 0.3, -0.7, 0.2, -0.4)
                #     rospy.sleep(0.1)
		
		        # Need to adjust nao position slightly at this point (0.06m forward, 0.03m left)

                while True:
                    if self.key == "C":
                        break

                    self.set_joint_angles(-0.3, 0.3, 0.3, -0.6, 0.2, -0.4)
                    rospy.sleep(0.1)

                # Main RL-DT loop
                cumm_reward = []
                total_reward = 0
                for episode in range(NUM_EPISODES):

                    # Get action from optimal policy
                    action, pause = env.get_action(nao.state)

                    # Take action
                    RHipRoll, RHipPitch = nao.action_execution(env, action)

                    if pause:
                        self.set_joint_angles(-0.3, 0.3, 0.3, RHipRoll, RHipPitch, -0.3) # heel in way
                        while True:
                            if self.key == "G" or self.key == "M" or self.key == "F":
                                break
                    else:
                        self.set_joint_angles(-0.3, 0.3, 0.3, RHipRoll, RHipPitch, -0.4)
                    
                    rospy.sleep(0.1)

                    # Determine next state
                    state_ = self.determine_state(env)

                    # Determine reward from action taken
                    reward = env.get_reward(self.key)
                    total_reward += reward
                    cumm_reward.append(total_reward)                   
                    
                    # Update model
                    env.train(nao.state, action, state_, reward)

                    # Update state
                    nao.state = state_

                    if pause:
                        while True:
                            if self.key == "C":
                                break

                            self.set_joint_angles(-0.3, 0.3, 0.3, -0.6, 0.2, -0.4)
                            rospy.sleep(0.1)

                        # Reset
                        nao.state = np.zeros(2)
                        nao.a1 = 8
                        nao.a2 = 0
                        self.key = ""
                    elif self.key == "Q":
                        break
                cumm_reward = np.array(cumm_reward)
                show_plot(cumm_reward)
                save_environment(env)

        else:
            # Load environment (contains Q & decision trees)
            env = load_environment()

            while True:

                while True:
                    if self.key == "C":
                        break

                    self.set_joint_angles(-0.3, 0.3, 0.3, -0.6, 0.2, -0.4)
                    rospy.sleep(0.1)

                rospy.sleep(1.0)

                # Get current state
                nao.state = self.determine_state(env)

                # Get action based on learned model
                action, pause = env.get_action(nao.state)

                # Take action
                RHipRoll, RHipPitch = nao.action_execution(env, action)

                if pause:
                    self.set_joint_angles(-0.3, 0.3, 0.3, RHipRoll, RHipPitch, -0.25) # heel in way
                    while True:
                        if self.key == "G" or self.key == "M" or self.key == "F":
                            break
                    
                    while True:
                        if self.key == "C":
                            break

                        self.set_joint_angles(-0.3, 0.3, 0.3, -0.6, 0.2, -0.4)
                        rospy.sleep(0.1)
                    
                    # Reset
                    nao.state = np.zeros(2)
                    nao.a1 = 0
                    nao.a2 = 0
                    self.key = ""
                elif self.key == "Q":
                    break
                else:
                    self.set_joint_angles(-0.3, 0.3, 0.3, RHipRoll, RHipPitch, -0.4)

                rospy.sleep(0.1)

        self.set_stiffness(False) # always check that your robot is in a stable position before disabling the stiffness!!

        rospy.spin()

        while not rospy.is_shutdown():
            self.set_stiffness(self.stiffness)
            rate.sleep()

    # each Subscriber is handled in its own thread
    #rospy.spin()

class Agent:

    def __init__(self, state, a1, a2):
        self.state = state
        self.a1 = a1 # RHipRoll action idx
        self.a2 = a2 # RHipPitch action idx
    
    def action_execution(self, env, action):
        self.a1 += env.action_translations[action][0]
        self.a2 += env.action_translations[action][1]
        # print(env.action_translations[action][1])
        
        # Restrict boundaries
        if self.a1 < 0:
            self.a1 = 0
        elif self.a1 > N_ACTIONS - 1:
            self.a1 = N_ACTIONS - 1

        # # Should not matter because pause & reset
        # if self.a2 < 0:
        #     self.a2 = 0
        # elif self.a1 > 1:
        #     self.a2 = 1
        
        # Get action for NAO execution
        RHipRoll = env.RHipRoll_actions[self.a1]
        RHipPitch = env.RHipPitch_actions[self.a2]

        return RHipRoll, RHipPitch

class Environment:

    def __init__(self, Ns_leg, Ns_gk):
        # Total number of states
        self.Ns_leg = Ns_leg
        self.Ns_gk = Ns_gk

        # State sets
        # self.Sm1 = []
        # self.Sm2 = []
        self.Sm1 = np.zeros(NS_LEG)
        self.Sm2 = np.zeros(NS_GK)

        # Define quantized action space
        self.RHipRoll_actions = np.linspace(-0.5, -0.75, N_ACTIONS) # Number hip roll actions
        self.RHipPitch_actions = np.array((0.2, -1.4))

        # Define actions
        self.action_dict = {"left": 0, "right": 1, "kick": 2}
        self.action_list = [0, 1, 2]
        self.action_translations = [(-1, 0), (1, 0), (0, 1)] # action translations within quantized action space

        # Define rewards
        self.goal_reward = 20 # Reward for scoring goal
        self.miss_penalty = -2 # Miss the goal
        self.fall_penalty = -20 # Penalty for falling over
        self.action_penalty = -1 # Penalty for each action execution
        
        # Learning parameters
        self.tau = 0.1
        self.alpha = 0.1
        self.gamma = 0.2 # Discount factor
        
        # Q values for state and action
        self.Q = np.zeros((self.Ns_leg, 
                           self.Ns_gk,
                           len(self.action_list)))

    def allowed_actions(self, s1):
        # Generate list of actions allowed depending on nao leg state
        actions_allowed = []
        if (s1 < self.Ns_leg - 2):  # No passing furthest left kick
            actions_allowed.append(self.action_dict["left"])
        if (s1 > 1):  # No passing furthest right kick
            actions_allowed.append(self.action_dict["right"])
        actions_allowed.append(self.action_dict["kick"]) # always able to kick
        actions_allowed = np.array(actions_allowed, dtype=int)
        return actions_allowed
    
    def get_reward(self, key):
        if key == "G":
            reward = self.goal_reward
        elif key == "M":
            reward = self.miss_penalty
        elif key == "F":
            reward = self.fall_penalty
        else:
            reward = self.action_penalty
        return reward
    
    def softmax(self, Q):
        Q /= self.tau
        Q_max = np.max(Q)
        num = np.exp(Q - Q_max)
        den = np.sum(num)
        prob = num / den
        return prob
                 
    def get_action(self, state):
        actions_allowed = self.allowed_actions(state[0])
        Q_sa = self.Q[state[0], state[1], actions_allowed]
        Q_sa_prob = self.softmax(Q_sa)

        action = np.random.choice(actions_allowed, p = Q_sa_prob)

        # print(state)
        print("Q_prob: {}".format(Q_sa_prob))
        print("Action: {}".format(action))

        pause = False
        if action == 2:
            pause = True # pause after kicking ball

        return action, pause
    
    def train(self, state, action, state_, reward):
        self.Q[state[0], state[1], action] = (1-self.alpha) * self.Q[state[0], state[1], action] + \
        self.alpha * (reward + self.gamma * np.max(self.Q[state_[0], state_[1], :] - \
                                                   self.Q[state[0], state[1], action]))

def yes_or_no(question):
    while "The answer is invalid":
        reply = str(raw_input(question+' (y/n): ')).lower().strip()
        if reply[:1] == 'y':
            return True
        if reply[:1] == 'n':
            return False

def show_plot(cumm_reward):
    plt.plot(cumm_reward)
    plt.xlabel("Episodes")
    plt.ylabel("Cummulative Reward")
    plt.grid()
    dirname = os.path.dirname(os.path.abspath(__file__))
    plt.savefig(os.path.join(dirname, "Cummulative_Reward-{}Eps.png".format(NUM_EPISODES)), dpi=500)
    plt.show()

def save_environment(env):
    dirname = os.path.dirname(os.path.abspath(__file__))

    env_fn = os.path.join(dirname, "environment.obj")
    pickle.dump(env, open(env_fn, 'wb'))

def load_environment():
    dirname = os.path.dirname(os.path.abspath(__file__))

    env_fn = os.path.join(dirname, "environment.obj")
    env = pickle.load(open(env_fn, 'rb'))
    return env
    

def main():
    env = Environment(Ns_leg=NS_LEG, Ns_gk=NS_GK)
    nao = Agent(state=np.zeros(2), a1=8, a2=0)

    # Instantiate central class and start loop
    central_instance = Central()
    central_instance.central_execute(nao, env)

if __name__=='__main__':
    main()

