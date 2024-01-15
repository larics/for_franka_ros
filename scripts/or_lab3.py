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

        # Init pose
        self.q_init = [-0.488, -0.641, 0.553, -2.17, -0.525, 3.19, 0.05]
        self.real_robot = False
        self.taylor_points = []

        if self.real_robot: 
            self.p_state_name = "/franka_state_controller/ee_pose"
            self.q_state_name = "/franka_state_controller/joint_states"
        else: 
            self.p_state_name = "/control_arm_node/tool/current_pose"
            self.q_state_name = "/joint_states"
            self.joint_names = ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6",
                                "panda_joint7"]
        
 

        # Initialize subscribers and publishers
        self._init_publishers()
        self._init_subscribers()
        self._init_srv_clients()
        self.p_reciv = False
        self.q_reciv = False

    def _init_subscribers(self): 
        # P and Q subscribers
        self.p_state_sub = rospy.Subscriber(self.p_state_name, Pose, self.pose_cb, queue_size=1)
        self.q_state_sub = rospy.Subscriber(self.q_state_name, JointState, self.joint_state_cb, queue_size=1)

    def _init_publishers(self): 
        # Trajectory publisher
        self.q_cmd_pub = rospy.Publisher("/position_joint_trajectory_controller/command", JointTrajectory, queue_size=1)

    def _init_srv_clients(self):
        # Service for fetching IK
        rospy.wait_for_service("/control_arm_node/services/get_ik")
        self.get_ik_client = rospy.ServiceProxy("/control_arm_node/services/get_ik", getIk)
        rospy.loginfo("Inverse kinematics initialized!")

    def pose_cb(self, data):
        # Get end effector pose
        self.p_reciv = True
        self.p_curr = PoseStamped()
        self.p_curr.pose.position = data.position
        self.p_curr.pose.orientation = data.orientation 
        
    def joint_state_cb(self, data): 
        # Get joint states
        self.q_reciv = True
        self.q_curr = JointState()
        self.q_curr.header = data.header
        self.q_curr.name = data.name
        positions = data.position
        self.q_curr.position = positions[:-2]
        self.q_curr.velocity = data.velocity
        self.q_curr.effort = data.effort

    def get_ik(self, wanted_pose):

        if isinstance(wanted_pose, np.ndarray):
            wanted_pose = arrayToPose(wanted_pose)
        try:
            response = self.get_ik_client(wanted_pose, self.p_curr.pose)
            q_ = np.array(response.jointState.position)
            return q_
        except rospy.ServiceException as e:
            rospy.logwarn("Service call failed: {}".format(e))
            return False
        
    def transform_points(self): 
        pass

    # Second lab HoCook
    def taylor_interpolate_point(self, start_pose, end_pose, epsilon):
        # Get q0, q1
        q0 = self.get_ik(arrayToPose(start_pose))
        q1 = self.get_ik(arrayToPose(end_pose))
        # get qm
        qm = calc_joint_midpoint(q0, q1)
        # calculate w_m
        p_wm = poseToArray(forwardKinematics(qm))
        # calculate w_M
        p_wM = calc_cartesian_midpoint(arrayToPose(start_pose), arrayToPose(end_pose))
        n = norm(p_wm, p_wM)
        if (n < epsilon):
            rospy.loginfo("Taylor interpolation finished")
            # It can be added nicer for sure! 
            self.taylor_points.append([np.round(s, 7) for s in start_pose]); 
            self.taylor_points.append([np.round(n, 7) for n in p_wM])
            #self.taylor_points.append([np.round(e, 5) for e in end_pose]) 
        else:
            rospy.loginfo("Taylor interpolation, condition not satisfied")
            return self.taylor_interpolate_points([start_pose, p_wM, end_pose], epsilon)

    def taylor_interpolate_points(self, poses_list, epsilon):
        for i, p in enumerate(poses_list):
            if i < len(poses_list) - 1: 
                self.taylor_interpolate_point(p, poses_list[i+1], epsilon)
        rospy.logdebug("Completed taylor: \n {}".format(self.taylor_points))
          
        return self.taylor_points
    
    def ho_cook(self, cartesian, t = None, q=None, first = True):

        # List of joint positions that must be visited
        if first:
            q = [self.get_ik(arrayToPose(pos)) for pos in cartesian]
            #print("Inverse kinematics q is: {}".format(q))
            # Time parametrization
            t = get_time_parametrization(q)
            print("Initial parametrization is: {}".format(t))

        n = len(self.joint_names)
        # Ap matrix
        Ap = createApmatrix(n, q, t)
        #print("Ap [{}] is: {}".format(Ap.shape, Ap))
        Mp = createMpmatrix(t)
        #print("Mp [{}] is: {}".format(Mp.shape, Mp))
        A = createAmatrix(n, q, t, Ap)
        #print("A [{}] is: {}".format(A.shape, A))
        M = createMmatrix(Mp, t)
        #print("M [{}] is: {}".format(M.shape, M))
        dq = np.matmul(A, np.linalg.inv(M))
        #print("dq [{}] is: {}".format(dq.shape, dq))
        zeros = np.zeros((self.n_joints, 1)); dq = np.hstack((zeros, dq)); dq = np.hstack((dq, zeros))
        #print("q len is: {}".format(len(q)))
        dq_max = get_dq_max(q, dq, t)
        dq_max_val = np.max(dq_max) 
        scaling_factor = dq_max_val/self.joint_max
        # Scale to accomodate limits
        t = [round(t_*scaling_factor, 5) for t_ in t]

        if scaling_factor > 1.0:
            print("Scaling factor is: {}".format(scaling_factor))
            return self.ho_cook(cartesian, t, q, first=False)

        else:
            trajectory = createTrajectory(self.joint_names, q, dq, t)
            rospy.loginfo("Publishing trajectory!")
            self.q_cmd_pub.publish(trajectory)
            sum_t = sum(t)
            rospy.loginfo("Execution duration is : {}".format(sum_t))
            return sum_t

    def go_to_pose_ho_cook(self, goal_pose, eps):

        # Reset saved ee_points and fk points
        self.ee_points = []; self.ee_points_fk = []
        start_time = rospy.Time.now().to_sec()
        # Calculate Taylor points
        points = self.taylor_interpolate_points([poseToArray(self.p_curr.pose), poseToArray(goal_pose)], eps)
        # TODO: Normalize quaternion before hocook 
        # Add time parametrization
        exec_duration = self.ho_cook(points)
        rospy.sleep(exec_duration)
        #draw(self.ee_points, self.ee_points_fk, points, eps)
        return points
    
    def go_to_init_pose(self): 
        rospy.loginfo("Going to init pose")
        trajectory = createSimpleTrajectory(self.joint_names, self.q_curr.position, self.q_init, 5)
        self.q_cmd_pub.publish(trajectory)

    def go_to_points(self, poses, sleep_time, t_move): 
        #TODO: Use methods from the run() in or_lab1.py to enable 
        # point to point movement for particular points to draw a house 
        for i, p in enumerate(poses):
            rospy.loginfo("Sending robot to pose {}: {}".format(i, p))
            q_start, q_end = JointTrajectoryPoint(), JointTrajectoryPoint() 
            trajectory = createSimpleTrajectory(self.joint_names, self.q_curr.position, self.get_ik(p), t_move)
            self.q_cmd_pub.publish(trajectory)
            rospy.sleep(sleep_time)
    

    # TODO: Add trajectory creation
        
    def run(self):

       pass



if __name__ == "__main__": 

    orLab = OrLab3();
    orLab.run()