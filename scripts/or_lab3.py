#!/usr/bin/env python3
# _author_: Filip Zorić; filip.zoric@fer.hr

import os
import rospy
import numpy as np
import matplotlib.pyplot as plt

from utils import *
from math import sqrt
from tf import TransformListener
from rospkg import RosPack

from tf.transformations import quaternion_from_matrix
from geometry_msgs.msg import PoseStamped, Pose, Point
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from mpl_toolkits.mplot3d import Axes3D
from std_srvs.srv import Trigger
from for_franka_ros.srv import getIk

class OrLab3(self): 

    def __init__(self): 
        rospy.init_node("orlab3", anonymous=True, log_level=rospy.INFO)
        self.tf_listener = TransformListener()
        self.ee_frame_name = "panda_hand_tcp"

        # Extract poses
        rp = RosPack()        
        yml_ = os.path.join(rp.get_path("for_franka_ros"), "config/goal_poses.yaml")
        poses_data = read_yaml_file(yml_)
        self.poses = get_poses(poses_data)
        self.pose_A = self.poses[0]
        self.pose_B = self.poses[1]
        self.pose_C = self.poses[2]
        self.pose_D = self.poses[3]
        self.pose_E = self.poses[4]

        # Initialize subscribers and publishers
        self._init_publishers()
        self._init_subscribers()
        self.pose_reciv = False

    def _init_subscribers(self): 

        self.p_state_sub = rospy.Subscriber("/franka_state_controller/ee_pose", Pose, self.pose_cb, queue_size=1)


    def _init_publishers(self): 

        self.p_cmd_pub = rospy.Publisher("/cartesian_impedance_controller/desired_pose")

    def pose_cb(self, data):
        
        self.pose_reciv = True
        self.current_pose = Pose()
        self.current_pose.position = data.position
        self.current_pose.orientation = data.orientation 

    def transform_points(self): 
        pass

    def point_to_point_movement(self): 
        #TODO: Use methods from the run() in or_lab1.py to enable 
        # point to point movement for particular points to draw a house 
        pass

    def ho_cook_movement(self): 
        # TODO: Use methods from the run() in or_lab2.py to enable 
        # following straight paths from point to point to draw nice house 
        pass
        

    def run():

        rospy.sleep(5)
        
        while not rospy.is_shutdown():
            pass
            # Point to point movement
            # self.point_to_point_movement()
            # Ho cook movement 




if __name__ == "__main__": 

    pass