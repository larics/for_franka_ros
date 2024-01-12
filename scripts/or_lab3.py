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

class OrLab3(): 

    def __init__(self): 
        # Node initialization 
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
        self._init_srv_clients()
        self.p_reciv = False
        self.q_reciv = False

    def _init_subscribers(self): 

        self.p_state_sub = rospy.Subscriber("/franka_state_controller/ee_pose", Pose, self.pose_cb, queue_size=1)
        self.q_state_sub = rospy.Subscriber("/franka_state_controller/joint_states", JointState, queue_size=1)

    def _init_publishers(self): 

        self.q_cmd_pub = rospy.Publisher("/position_joint_trajectory_controller/command", JointTrajectory, queue_size=1)

    def _init_srv_clients(self):
        # Service for fetching IK
        rospy.wait_for_service("/control_arm_node/services/get_ik")
        self.get_ik_client = rospy.ServiceProxy("/control_arm_node/services/get_ik", getIk)
        rospy.loginfo("Inverse kinematics initialized!")

    def pose_cb(self, data):
        
        self.p_reciv = True
        self.p_curr = PoseStamped()
        self.p_curr.pose.position = data.position
        self.p_curr.pose.orientation = data.orientation 
        
    def joint_state_cb(self, data): 

        self.q_reciv = True
        self.q_curr = JointState()
        self.q_curr.header = data.header
        self.q_curr.name = data.name
        self.q_curr.position = data.position
        self.q_curr.velocity = data.velocity
        self.q_curr.effort = data.effort

    def get_ik(self, wanted_pose):

        if isinstance(wanted_pose, np.ndarray):
            wanted_pose = arrayToPose(wanted_pose)
        try:
            response = self.get_ik_client(wanted_pose, self.current_pose)
            q_ = np.array(response.jointState.position)
            return q_
        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed: {}".format(e))
            return False
        
    def transform_points(self): 
        pass

    def point_to_point_movement(self, sleep_time, movement_time): 
        #TODO: Use methods from the run() in or_lab1.py to enable 
        # point to point movement for particular points to draw a house 
        for i, p in enumerate(self.poses):
            rospy.loginfo("Sending robot to pose {}: {}".format(i, p))
            q_start, q_end = JointTrajectoryPoint(), JointTrajectoryPoint()
            q_start.positions = self.q_curr.position 
            q_start.time_from_start = 0
            q_end.positions = self.get_ik(p)
            q_end.time_from_start = movement_time
            #t_i = createSimpleTrajectory()
            rospy.sleep(sleep_time)

    def ho_cook_movement(self): 
        # TODO: Use methods from the run() in or_lab2.py to enable 
        # following straight paths from point to point to draw nice house 
        pass

    # TODO: Add trajectory creation
        
    def run(self):

        rospy.sleep(5)
        
        while not rospy.is_shutdown():
            if self.p_reciv and self.q_reciv:
                # Point to point movement
                self.point_to_point_movement(5, 5)
            else: 
                rospy.logwarn("Recieved p: {} \t Recieved q: {}".format(self.p_reciv, self.q_reciv))
            # Ho cook movement 



if __name__ == "__main__": 

    orLab = OrLab3();
    orLab.run()