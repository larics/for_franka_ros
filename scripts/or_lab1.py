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
    p = forwardKinematics(q)

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

# Return goal position based on the forwardKinematics? 
def forwardKinematics(q):
    q1 = q[0]; q2 = q[1]; q3 = q[2]; q4 = q[3]; q5 = q[4]; q6 = q[5]; q7 = q[6]

    ## TODO: 1st! Insert your kinematic parameters here
    T_0_1 = TfromDH(q1, 0, np.pi, 0)
    T_1_2 = TfromDH(q2, 0, 0, 0)
    # ... rest of the transformation
    # Matrix multiplication of the defined transformation matrices 
    # hint1: use matmul, and correct order of multiplication to obtain 
    # transformation from the base link to the end effector
    # hint2: current matmul order/example may be incorrect, it is simply placeholder
    # TODO: Write matrix multiplication expression to obtaion transformation from zero frame (base)
    # to the tool / end_effector
    T_0_e = np.matmul(T_0_1, T_1_2) 
    #...
    # Return goal position from T0_e
    return poseFromMatrix(T_0_e)


class OrLab1():
    def __init__(self):
        rospy.init_node("orlab1", anonymous=True, log_level=rospy.INFO)
        self.current_pose = Pose()
        self.init_pose = Pose()
        self.init_pose.position.x = 0.31; self.init_pose.position.y = 0.0; self.init_pose.position.z = 0.49; 
        self.init_pose.orientation.x = -1; self.init_pose.orientation.y = 0.0; self.init_pose.orientation.z = 0; self.init_pose.orientation.w = 0; 
        self.start_pose = Pose()
        self.start_pose.position.x = 0.42; self.start_pose.position.y = 0.03; self.start_pose.position.z = 0.65; 
        self.start_pose.orientation.x = -0.705; self.start_pose.orientation.y = 0.18; self.start_pose.orientation.z = -0.575; self.start_pose.orientation.w = 0.315; 
        self.pose_list = []
        self.tf_listener = TransformListener()
        self._init_subs()
        self._init_pubs()
        self.ee_frame_name = "panda_hand_tcp"
        self.ee_points = []
        self.ee_points_fk = []
        #self.poses = [self.p1, self.p2, self.p3, self.p4, self.p5]

        Q1 = []; 
        Q2 = []; 
        Q3 = []; 
        Q4 = []; 
        Q5 = []; 
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
        q_s = msg.position
        try:
            (t,q) = self.tf_listener.lookupTransform("world", self.ee_frame_name, rospy.Duration(0))
        except (LookupException, ConnectivityException, ExtrapolationException):
            return
        if len(self.ee_points) == 0:
            self.ee_points.append(np.asarray([t[0], t[1], t[2]]))
            self.ee_points_fk.append(plotFK(q_s))
        else:
            new_p = np.asarray([t[0], t[1], t[2]])
            last_p = self.ee_points[-1]
            if np.linalg.norm(new_p-last_p) > 0.0001:
                self.ee_points.append(new_p)
                self.ee_points_fk.append(plotFK(q_s))

    def sendRobotToPose(self, matrix):
        self.pose_pub.publish(poseFromMatrix(matrix))
        rospy.sleep(5)
        
    def sendRobotToInitPose(self): 
        self.pose_pub.publish(self.init_pose)
        rospy.sleep(5)
        
    def run(self):
        self.sendRobotToInitPose()

        ## reconstruct T_world_p1
        # T_b_1 = forwardKinematics(Q1)
        # T_b_2 = forwardKinematics(Q2)
        # T_b_3 = forwardKinematics(Q3)
        # T_b_4 = forwardKinematics(Q4)
        # T_b_5 = forwardKinematics(Q5)
        Ts = [T_b_1, T_b_2, T_b_3, T_b_4, T_b_5]

        # self.pose_pub.publish(poseFromMatrix(T_b_1))
        # rospy.sleep(5)
        # self.pose_pub.publish(poseFromMatrix(T_world_bottlegrasp))

        # TODO: []
        self.poses = [poseFromMatrix(T) for T in Ts]
        order = []

        for i in order: 
            rospy.loginfo("Visiting {} point".format(i))
            self.pose_pub.publish(self.poses[i-1])
            rospy.sleep(15)     

        self.sendRobotToInitPose()
        ## send robot back to init pose
        # Draw points
        draw(self.ee_points, self.ee_points_fk)


if __name__ == "__main__":
    lab1 = OrLab1()
    
    while not rospy.is_shutdown():
        try:
            lab1.run()
        except KeyboardInterrupt: 
            break
