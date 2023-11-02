#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped, Pose, Point
import numpy as np
from tf.transformations import quaternion_from_matrix
from tf import TransformListener, LookupException, ConnectivityException, ExtrapolationException
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import copy
import math

def normalize_q(pose):
    qx = pose.orientation.x
    qy = pose.orientation.y 
    qz = pose.orientation.z 
    qw = pose.orientation.w  
    norm = np.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
    qx = qx/norm; qy = qy/norm; qz = qz/norm; qw = qw/norm
    pose.orientation.x = qx 
    pose.orientation.y = qy 
    pose.orientation.z = qz 
    pose.orientation.w = qw 
    return pose

def poseFromMatrix(matrix):
    goal_pose = Pose()
    quat = quaternion_from_matrix(matrix)
    goal_pose.position.x = matrix[0,3]
    goal_pose.position.y = matrix[1,3]
    goal_pose.position.z = matrix[2,3]
    goal_pose.orientation.x = quat[0]
    goal_pose.orientation.y = quat[1]
    goal_pose.orientation.z = quat[2]
    goal_pose.orientation.w = quat[3]
    return goal_pose

def TfromDH(theta, d, alpha, a):
    T = np.eye(4)
    T[0,0] = np.cos(theta)
    T[0,1] = -np.sin(theta)*np.cos(alpha)
    T[0,2] = np.sin(theta)*np.sin(alpha)
    T[0,3] = a*np.cos(theta)
    T[1,0] = np.sin(theta)
    T[1,1] = np.cos(theta)*np.cos(alpha)
    T[1,2] = -np.cos(theta)*np.sin(alpha)
    T[1,3] = a*np.sin(theta)
    T[2,0] = 0
    T[2,1] = np.sin(alpha)
    T[2,2] = np.cos(alpha)
    T[2,3] = d
    return T

def plotFK(q): 
    p = forwardKinematics(q, plot=True)
    return [p.position.x, p.position.y, p.position.z]

def draw(points_gazebo, points_fk):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    arr_points = np.stack(points_gazebo, axis=0)
    ax.plot(arr_points[:,0],arr_points[:,1],arr_points[:,2], label='Gazebo path')
    fk_points = np.stack(points_fk, axis=0)
    ax.plot(fk_points[:,0],fk_points[:,1],fk_points[:,2], 'r', label='Forward kinematics')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_zlabel('z [m]')
    ax.legend(loc='lower right')
    plt.title('End effector path')
    plt.show()

def forwardKinematics(q, plot=False):
    """ Forward kinematics 

    Args:
        q (np.array): measured joint values in the array 7x1
        plot (bool): False

    Returns:
        T0e (np.matrix): HTM of the tool in the 0 frame [world frame]
    """

    q1 = round(q[0], 4); q2 = round(q[1], 4); q3 = round(q[2], 4); q4 = round(q[3], 4); q5 = round(q[4], 4); q6 = round(q[5],4); q7 = round(q[6],4)

    M_PI = math.pi
    # TODO: 1st! Insert your kinematic parameters here
    # theta, d, alpha, a
    theta = 0; d = 0; alpha = 0; a = 0; 
    T01 = TfromDH(theta, d, alpha, a)
    
    # TODO: Write matrix multiplication term that returns transformation 
    # from the base_link (0) to the end effector e, use np.matmul or np.dot
    # check numpy documentation! 
    T0e = np.eye(4, 4)

    return poseFromMatrix(T0e)

def getTfromEuler(t, euler_vector):
    """ Return homogenous transformation matrix (HTM) from translation and the euler vector.

    Args:
        t (np.array): translation vector
        euler_vector (np.array): rotation vector written as euler angles
    Returns: 
        T (np.matrix): homogenous transformation matrix
    """

    pass

    return T

def get_T(T_point, T_BW, T_WT, T_BT): 
    """Return transformed point based on the given HTMs.

    Args:
        T_point (np.matrix): HTM of the point that needs to be shown in another coordinate frame
        T_BW (np.matrix): HTM of the base in the world frame
        T_WT (np.matrix): HTM of the world in the target frame
        T_BT (np.matrix): HTM of the target in the base frame
    Returns: 
        nT_point (np.matrix): HTM of the transformed point in the new coordinate frame
    """

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
        self._init_subs()
        self._init_pubs()
        self.ee_frame_name = "panda_hand_tcp"
        self.ee_points = []
        self.ee_points_fk = []
        self.msg_reciv = False
        # Define poses for visiting! 
        self.p1 = Pose(); 
        self.p1.position.x = 0.4
        self.p1.position.y = 0.0 
        self.p1.position.z = 0.6
        self.p1.orientation.x = -0.692
        self.p1.orientation.y = 0.203
        self.p1.orientation.z = -0.6078
        self.p1.orientation.w = 0.331
        normalize_q(self.p1); 
        self.p2 = copy.deepcopy(self.p1); 
        self.p2.position.z = 0.7; 
        self.p3 = copy.deepcopy(self.p2)
        self.p3.position.y = -0.1
        self.p3.position.z = 0.6
        self.p4 = copy.deepcopy(self.p3)
        self.p4.position.z = 0.7
        self.p5= copy.deepcopy(self.p4)
        self.p5.position.y = -0.05
        self.p5.position.z = 0.8

        # Poses that represent small house
        self.poses = [self.p1, self.p2, self.p3, self.p4, self.p5]

        # Poses in the joint space
        self.Q1 = [0.10498037158297269, -0.8661767348888807, -0.12536473661681047, -2.2633746616581547, 0.6878899307541717, 2.84523589989211, 0.796144453328]; 
        self.Q2 = [0.60195205214578, -1.0176603655579513, -0.26678371589446215, -2.794013825558414, 0.8293323483622554, 3.481340900004292, 0.5449582738253334]; 
        self.Q3 = [-0.046799258479464534, -0.9061714256000792, 0.08583176237453838, -2.383377932774004, 1.0529403405705002, 2.8997051316198323, 0.5878432032726977]; 
        self.Q4 = [0.4199388644517912, -1.0627829105268898, -0.3803688797238589, -2.6435792063661294, 1.3298469531070012, 3.169200341336367, -0.0913129143323328]; 
        self.Q5 = [0.08187246737016451, -0.9330090681099925, -0.19852447581569255, -2.454975092541262, 1.1114599324370698, 2.871296585762611, 0.2974855792073763]; 
        self.Q = [self.Q1, self.Q2, self.Q3, self.Q4, self.Q5]

        rospy.sleep(0)

    def _init_pubs(self):
        self.pose_pub = rospy.Publisher("/control_arm_node/arm/command/pose", Pose, queue_size=10, latch=True)
        rospy.loginfo("Initialized publishers!")

    def _init_subs(self):
        self.pose_sub = rospy.Subscriber("/control_arm_node/tool/current_pose", Pose, self.tool_cb, queue_size=1)
        self.joint_states_sub = rospy.Subscriber("/joint_states",JointState, self.joints_cb, queue_size=1)
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
            for i in order: 
                rospy.loginfo("Visiting {} point".format(i))
                forwardKinematics(self.q_s, plot=False)
                self.sendRobotToPose(self.poses[i], 10)    

            self.sendRobotToInitPose()
            draw(self.ee_points, self.ee_points_fk)


if __name__ == "__main__":
    lab1 = OrLab1()
    
    while not rospy.is_shutdown():
        try:
            lab1.run()
        except KeyboardInterrupt: 
            break
