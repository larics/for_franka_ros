#!/usr/bin/env python3
# _author_: Filip Zorić; filip.zoric@fer.hr

import os
import rospy
import numpy as np
import matplotlib.pyplot as plt

from utils import *
from math import sqrt
from rospkg import RosPack

from tf import TransformListener, LookupException, ConnectivityException, ExtrapolationException
from tf.transformations import quaternion_from_matrix
from geometry_msgs.msg import PoseStamped, Pose, Point
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from mpl_toolkits.mplot3d import Axes3D
from std_srvs.srv import Trigger
from for_franka_ros.srv import getIk

class OrLab(): 

    def __init__(self, node_name, frequency): 
        rospy.init_node(f"{node_name}", anonymous=True, log_level=rospy.INFO)
        self.frequency = frequency
        pass
    
    def _init_subscribers(self):

        pass

    def _init_publishers(self): 
        self.p_pub = rospy.Publisher("/control_arm_node/arm/command/pose", Pose, queue_size=10, latch=True)
        self.q_pub = rospy.Publisher("")
        pass

    def _init_services(self): 
        pass

    def joint_state_cb(self): 
        pass

    def ee_state_cb(self): 
        pass

    def run(self): 
        while not rospy.is_shutdown(): 
            rospy.sleep(rospy.Duration(1/int(self.frequency)))

if __name__=="__main__": 
    pass


