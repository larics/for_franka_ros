#!/usr/bin/env python3

import rospy
import copy
import math
import numpy as np

from geometry_msgs.msg import PoseStamped, Pose, Point
from tf.transformations import quaternion_from_matrix
from tf import TransformListener, LookupException, ConnectivityException, ExtrapolationException
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from utils import *

def getTfromEuler(t, euler_vector):
    """ Return homogenous transformation matrix (HTM) from translation and the euler vector.

    Args:
        t (np.array): translation vector [3x1]
        euler_vector (np.array): rotation vector written as euler angles [3x1]
    Returns: 
        T (np.matrix): homogenous transformation matrix [4x4]
    """

    pass

    return T

def get_T(p, T_BW, T_WT, T_BT): 
    """Return transformed point based on the given HTMs.

    Args:
        point (np.array): point that will transformed into other coordinate frame [4x1]
        T_BW (np.matrix): HTM of the base in the world frame [4x4]
        T_WT (np.matrix): HTM of the world in the target frame [4x4]
        T_BT (np.matrix): HTM of the target in the base frame [4x4]
    Returns: 
        nT_point (np.array): Transformed point in the new coordinate frame [4x1]
    """

    T_point = np.array([p.position.x, p.position.y, p.position.z, 1]).reshape(4, 1)

    pass

    return nT_point

class OrLab1():
    def __init__(self):
        rospy.init_node("orlab1", anonymous=True, log_level=rospy.INFO)
        self.current_pose = Pose()
        self.init_pose = Pose()
        self.init_pose.position.x = 0.31; self.init_pose.position.y = 0.0; self.init_pose.position.z = 0.49; 
        self.init_pose.orientation.x = -1; self.init_pose.orientation.y = 0.0; self.init_pose.orientation.z = 0; self.init_pose.orientation.w = 0; 
        self.pose_list = []
        self.tf_listener = TransformListener()
        self._init_subs(); self._init_pubs()
        self.ee_frame_name = "panda_hand_tcp"
        self.ee_points = []; self.ee_points_fk = []
        self.msg_reciv = False
        
        # Define poses for visiting! 
        self.p1 = Pose(); 
        self.p1.position.x = 0.4; self.p1.position.y = 0.0; self.p1.position.z = 0.6
        self.p1.orientation.x = -0.692; self.p1.orientation.y = 0.203; self.p1.orientation.z = -0.6078; self.p1.orientation.w = 0.331
        normalize_q(self.p1); 
        self.p2 = copy.deepcopy(self.p1); self.p2.position.z = 0.7; 
        self.p3 = copy.deepcopy(self.p2); self.p3.position.y = -0.1; self.p3.position.z = 0.6
        self.p4 = copy.deepcopy(self.p3); self.p4.position.z = 0.7
        self.p5 = copy.deepcopy(self.p4); self.p5.position.y = -0.05; self.p5.position.z = 0.8

        # Poses that represent small house
        self.poses = [self.p1, self.p2, self.p3, self.p4, self.p5]

        # Poses in the joint space
        self.Q1 = [0.104, -0.866, -0.125, -2.263, 0.687, 2.845, 0.796]; 
        self.Q2 = [0.601, -1.017, -0.266, -2.794, 0.829, 3.481, 0.544]; 
        self.Q3 = [-0.046, -0.906, 0.085, -2.383, 1.052, 2.899, 0.587]; 
        self.Q4 = [0.419, -1.062, -0.380, -2.643, 1.329, 3.169, -0.091]; 
        self.Q5 = [0.081, -0.933, -0.198, -2.454, 1.111, 2.871, 0.297]; 
        self.Q = [self.Q1, self.Q2, self.Q3, self.Q4, self.Q5]

        rospy.sleep(0)

    def _init_pubs(self):
        self.pose_pub = rospy.Publisher("/control_arm_node/arm/command/pose", Pose, queue_size=10, latch=True)
        rospy.loginfo("Initialized publishers!")

    def _init_subs(self):
        self.pose_sub = rospy.Subscriber("/control_arm_node/tool/current_pose", Pose, self.tool_cb, queue_size=1)
        self.joint_states_sub = rospy.Subscriber("/joint_states", JointState, self.joints_cb, queue_size=1)
        rospy.loginfo("Initialized subscribers!") 

    ## Callbacks
    def tool_cb(self, msg):
        self.current_pose.position = msg.position
        self.current_pose.orientation = msg.orientation

    def joints_cb(self, msg):
        self.msg_reciv = True
        self.q_s = msg.position
        try:
            (t, q) = self.tf_listener.lookupTransform("world", self.ee_frame_name, rospy.Duration(0))
        except (LookupException, ConnectivityException, ExtrapolationException):
            return
        if len(self.ee_points) == 0:
            self.ee_points.append(np.asarray([t[0], t[1], t[2]]))
            self.ee_points_fk.append(plotFK(self.q_s))
        else:
            new_p = np.asarray([t[0], t[1], t[2]])
            last_p = self.ee_points[-1]
            if np.linalg.norm(new_p-last_p) > 0.0001:
                self.ee_points.append(new_p)
                self.ee_points_fk.append(plotFK(self.q_s))

    def sendRobotToPose(self, pose, sleepT):
        self.pose_pub.publish(pose)
        rospy.sleep(sleepT)
        
    def sendRobotToInitPose(self): 
        self.pose_pub.publish(self.init_pose)
        rospy.sleep(5)
        
    def run(self):

        if self.msg_reciv: 
            self.sendRobotToInitPose()
            order = [0, 1, 2, 3, 1, 4, 3, 0, 2]
            transformedT = []
            for i in order: 
                rospy.loginfo("Visiting {} point".format(i))
                # TODO: Test FK that's implemented 
                # uncomment next line to test FK 
                # pose_i = forwardKinematics(self.Q[i], plot=False)
                # comment next line if testing FK
                pose_i = self.poses[i]
                self.sendRobotToPose(pose_i, 10)
                # TODO: Ater finishing FK, test gtT for transforming points    
                # transformedT.append(self.get_T(pose_i, ...)

            self.sendRobotToInitPose()
            # Draw FK
            draw(self.ee_points, self.ee_points_fk)
            # TODO: Uncomment to test draw trasformed points
            # draw(self.ee_points, transformedT)


if __name__ == "__main__":
    lab1 = OrLab1()
    
    while not rospy.is_shutdown():
        try:
            lab1.run()
        except KeyboardInterrupt: 
            break
