#!/usr/bin/env python3
# _author_: Filip Zorić; filip.zoric@fer.hr

import os
import rospy
import copy
import numpy as np
from math import sqrt

from tf.transformations import quaternion_from_matrix
from tf import TransformListener, LookupException, ConnectivityException, ExtrapolationException
from geometry_msgs.msg import PoseStamped, Pose, Point
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from std_srvs.srv import Trigger
from for_franka_ros.srv import getIk
from rospkg import RosPack
from utils import *


class OrLab2():
    def __init__(self):
        rospy.init_node("orlab2", anonymous=True, log_level=rospy.INFO)
        self.tf_listener = TransformListener()
        self.ee_frame_name = "panda_hand_tcp"

        # Read goal poses
        rp = RosPack()        
        yml_ = os.path.join(rp.get_path("for_franka_ros"), "config/goal_poses.yaml")
        poses_data = read_yaml_file(yml_)
        self.poses = get_poses(poses_data)
        self.pose_A = self.poses[0]
        self.pose_B = self.poses[1]
        self.pose_C = self.poses[4]

        # Helper vars
        self.ee_points = []
        self.ee_points_fk = []
        self.current_pose = Pose()
        self.cmd_eps = 0.002
        self.ee_frame_name = "panda_hand_tcp"
        self.joint_names = ["panda_joint1", "panda_joint2", "panda_joint3", "panda_joint4", "panda_joint5", "panda_joint6",
                             "panda_joint7"]
        self.n_joints = len(self.joint_names)

                # IK/FK Poses
        self.taylor_points = []
        self.joint_max = 30.0

        # Init methods
        self._init_subs()
        self._init_pubs()
        self._init_srv_clients()

    def _init_pubs(self):
        self.pose_pub = rospy.Publisher("/control_arm_node/arm/command/pose", Pose, queue_size=10, latch=True)
        self.q_cmd_pub = rospy.Publisher("/position_joint_trajectory_controller/command", JointTrajectory, queue_size=1)
        rospy.loginfo("Initialized publishers!")

    def _init_subs(self):
        self.pose_sub = rospy.Subscriber("/control_arm_node/tool/current_pose", Pose, self.tool_cb, queue_size=1)
        self.joint_states_sub = rospy.Subscriber("/joint_states", JointState, self.joint_state_cb, queue_size=1)
        rospy.loginfo("Initialized subscribers!")

    def _init_srv_clients(self):
        # Service for fetching IK
        rospy.wait_for_service("/control_arm_node/services/get_ik")
        self.get_ik_client = rospy.ServiceProxy("/control_arm_node/services/get_ik", getIk)
        rospy.loginfo("Inverse kinematics initialized!")

    def tool_cb(self, msg):

        self.current_pose.position = msg.position
        self.current_pose.orientation = msg.orientation

    def joint_state_cb(self, msg):

        q_s = msg.position
        # Slicing because fingers position is related to gripper
        self.q_curr = msg.position[:-2]
        try:
            (t,q) = self.tf_listener.lookupTransform("world", self.ee_frame_name, rospy.Duration(0))
        except (LookupException, ConnectivityException, ExtrapolationException):
            return
        if len(self.ee_points) == 0:
            self.ee_points.append(np.asarray([t[0], t[1], t[2]]))
            self.ee_points_fk.append(poseToArray(forwardKinematics(q_s)))
        else:
            new_p = np.asarray([t[0], t[1], t[2]])
            last_p = self.ee_points[-1]
            if np.linalg.norm(new_p-last_p) > 0.0001:
                self.ee_points.append(new_p)
                self.ee_points_fk.append(poseToArray(forwardKinematics(q_s)))

    def calc_cartesian_midpoint(self, start_pose, end_pose):

        # Calculate position average
        x = (start_pose.position.x + end_pose.position.x)/2
        y = (start_pose.position.y + end_pose.position.y)/2
        z = (start_pose.position.z + end_pose.position.z)/2
        # Keep orientation same as in starting points
        qx = (start_pose.orientation.x)
        qy = (start_pose.orientation.y)
        qz = (start_pose.orientation.z)
        qw = (start_pose.orientation.w)

        return np.asarray([x, y, z, qx, qy, qz, qw])

    def calc_joint_midpoint(self, start_joint, end_joint):
        return (start_joint + end_joint)/2

    def norm(self, q_cmd, q_curr):
        norm = np.sqrt(np.sum((q_cmd[:3]- q_curr[:3])**2))
        return norm

    def execute_cmds(self, q_list):

        dt = 0.5
        t = 5
        if len(q_list) < 5:
            t = scale_sigmoid(10, len(q_list)/2)
        dt = t/len(q_list)
        # Publish calculated joint values
        qMsg = JointTrajectory()
        qMsg.joint_names = self.joint_names
        for i, q in enumerate(q_list):
            qMsgPoint = JointTrajectoryPoint()
            q_ = list(q); 
            qMsgPoint.positions = q_
            qMsgPoint.time_from_start = rospy.Duration.from_sec(dt + i*dt)
            qMsg.points.append(qMsgPoint)
        
        self.q_cmd_pub.publish(qMsg)
        rospy.sleep(rospy.Duration(t))

    def execute_cmd(self, q):
        qMsg = JointTrajectory()
        qMsg.joint_names = self.joint_names
        q = list(q); q.append(0); q.append(0)
        q_p = JointTrajectoryPoint()
        q_p.positions = q
        qMsg.points = q_p
        print(qMsg)
        self.q_cmd_pub.publish(qMsg)

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

    def taylor_interpolate_point(self, start_pose, end_pose, epsilon):
        # Get q0, q1
        q0 = self.get_ik(arrayToPose(start_pose))
        q1 = self.get_ik(arrayToPose(end_pose))
        # get qm
        qm = self.calc_joint_midpoint(q0, q1)
        # calculate w_m
        p_wm = poseToArray(forwardKinematics(qm))
        # calculate w_M
        p_wM = self.calc_cartesian_midpoint(arrayToPose(start_pose), arrayToPose(end_pose))
        n = self.norm(p_wm, p_wM)
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
        
        print("Completed taylor: \n {}".format(self.taylor_points))
          
        return self.taylor_points

    def get_time_parametrization(self, q):

        t = []
        for k, qk in enumerate(q):
            try:
                tk_1 = np.sqrt(np.sum((q[k+1] - q[k])**2))
                t.append(tk_1)
            except Exception as e:
                pass

        rospy.loginfo("Finished time parametrization, segment num, m-1: {}".format(len(t)))

        return t

    def createMpmatrix(self, t):
        # Matrix dimensions are (m - 2) x (m - 4)
        # t dimensions are m-1

        m_1 = len(t)
        Mp = np.zeros((m_1 - 1, m_1 - 3))
        for i, t_ in enumerate(t):
            try:
                Mp[i, i] =  t[i+2]
                Mp[i + 1, i] = 2*(t[i+1] + t[i+2])
                Mp[i + 2, i] = t[i+1]

            except Exception as e:
                pass

        rospy.loginfo("Finished adding elements to the Mp matrix, dimensions (m-2) x (m-4) : {}".format(Mp.shape))

        return Mp

    def createMmatrix(self, Mp, t):
        # Matrix dimensions are (m - 2) x (m - 2)

        m_1 = len(t)
        M1col = np.zeros((m_1 - 1, 1))
        Mlcol = np.zeros((m_1 - 1, 1))

        M1col[0, 0] = 3 /t[0] + 2/t[1]
        M1col[1, 0] = 1 /t[1]
        Mlcol[-2, 0] = 1 /t[-2]
        Mlcol[-1, 0] = 2/t[-2] + 3/t[-1]

        M = np.hstack((M1col, Mp))
        M = np.hstack((M, Mlcol))

        return M

    def createApmatrix(self, q, t):
        # Matrix dimensions are n x (m - 4)
        # t dimensions are m-1

        n = self.n_joints
        m_1 = len(t)
        Ap = np.zeros((n, m_1 - 3))
        for i , t_ in enumerate(t):
            try:
                c = 3/(t[i+1] * t[i+2])
                bracket = t[i+1]**2*(q[i+2] - q[i+1]) + t[i+2]**2*(q[i+1] - q[i])
                Ap[:, i] = c * bracket
            except Exception as e:
                pass

        rospy.loginfo("Finished adding elements to the Ap matrix, dimensions n x (m-4) : {}".format(Ap.shape))

        return Ap

    def createAmatrix(self, q, t, Ap):
        # Matrix dimensions are n x (m - 2)
        # t dimensions are m-1

        n = self.n_joints
        A1col = 6/t[0]**2 * (q[1] - q[0]) + 3/t[1] * (q[2] - q[1])
        Alcol = 3/t[-1]**2 * (q[-1] - q[-2]) + 6/t[-1]**2*(q[-1] - q[-2])

        A = np.hstack((A1col.reshape(n, 1), Ap))
        A = np.hstack((A, Alcol.reshape(n, 1)))

        return A

    # Get B matrices
    def getBfirstSeg(self, q, dq, t):

        T = np.zeros((4, 5))
        T[0, 0] = 1;            T[0, 3] = -4/t[0]**3;   T[0, 4] = 3/t[0]**4
        T[1, 3] = 4/t[0]**3;    T[1, 4] = -3/t[0]**4;
        T[3, 3] = -1/t[0]**2;   T[3, 4] = 1/t[0]**3;

        Q = np.array((q[0], q[1], dq[:, 0], dq[:, 1])).T # 6 x 4

        # (6 x 4) x (4 x 5)
        BfirstSeg = np.matmul(Q, T) # 6 x 5 dimensions

        print("Bfirst: {}".format(BfirstSeg))

        return BfirstSeg

    def getBanySeg(self, q, dq, t, k):

        T = np.zeros((4, 5))
        T[0, 0] = 1;            T[0, 3] = -3/t[k]**2;   T[0, 4] = 2/t[k]**3;
        T[1, 3] = 3/t[k]**2;    T[1, 4] = -2/t[k]**3;
        T[2, 1] = 1;            T[2, 2] = -2/t[k];      T[2, 3] = 1/t[k]**2;
        T[3, 2] = -1/t[k];      T[3, 3] = 1/t[k]**2;

        # Fix indexing (First seg, k=1, q[k-1] = q[0], but has to be q[1], q[2])
        k = k + 1
        Q = np.array((q[k-1], q[k], dq[:, k-1], dq[:, k])).T

        BkSeg = np.matmul(Q, T)

        return BkSeg

    def getBLastSeg(self, q, dq, t):

        T = np.zeros((4, 5))
        T[0, 0] = 1;            T[0, 2] = -6/t[-1]**2;  T[0, 3] = 8/t[-1]**2; T[0, 4] = -3/t[-1]**4;
        T[1, 2] = 6/t[-1]**2;   T[1, 3] = -8/t[-1]**3;  T[1, 4] = 3/t[-1]**4;
        T[2, 1] = 1;            T[2, 2] = -3/t[-1];     T[2, 3] = 3/t[-1]**2; T[2, 4] = -1/t[-1]**3;

        Q = np.array((q[-2], q[-1], dq[: ,-2], dq[:, -1])).T

        BlastSeg = np.matmul(Q, T)

        return BlastSeg

    def getMaxSpeedFirstSeg(self, B, t):

        q_max = B[:, 1] + 2 * B[:, 2]*t[0] + 3*B[:, 3]*t[0]**2 + 4*B[:, 4]*t[0]**3

        return q_max

    def getMaxSpeedAnySeg(self, B, t, k):

        q_max = B[:, 1] + 2*B[:, 2]*t[k] + 3*B[:, 3]*t[k]**2

        return q_max

    def getMaxSpeedLastSeg(self, B, t):

        q_max = B[:, 1] + 2*B[:, 2]*t[-1] + 3*B[:, 3]*t[-1]**2 + 4*B[:, 4]*t[-1]**3

        return q_max

    def getMaxAcc(self, B, t):

        q_dot_max = 2*B[:, 2] + 6*B[:, 3]*t

        return q_dot_max

    def createTrajectory(self, q, dq, t):

        trajectoryMsg = JointTrajectory()
        trajectoryMsg.joint_names = self.joint_names

        dq = list(dq.T)
        i = 0
        for k, (q, dq) in enumerate(zip(q, dq)):
            try:
                i += t[k]
                t_ = rospy.Time.from_sec(i)
            except Exception as e:
                t_ = rospy.Time.from_sec(np.ceil(i))
            trajectoryPoint = JointTrajectoryPoint()
            trajectoryPoint.positions = q
            trajectoryPoint.velocities = dq
            trajectoryPoint.time_from_start.secs = t_.secs
            trajectoryPoint.time_from_start.nsecs = t_.nsecs
            trajectoryMsg.points.append(trajectoryPoint)

        return trajectoryMsg

    def get_dq_max(self, q, dq, t):

        Bk = []; dqmax = []
        for k, t_ in enumerate(t):
            if k == 0:
                Bfirst = self.getBfirstSeg(q, dq, t)
                dqmax_ = self.getMaxSpeedFirstSeg(Bfirst, t)
                dqmax.append(dqmax_)
            # Calculate stuff for all segments without first > 0 and last len(t) - 1
            if k > 0 and k < len(t) - 1:
                Bk_ = self.getBanySeg(q, dq, t, k)
                Bk.append(Bk_)
                dqmax_ = self.getMaxSpeedAnySeg(Bk_, t,  k)
                dqmax.append(dqmax_)
            if k == len(t) - 1 :
                Blast = self.getBLastSeg(q, dq, t)
                dqmax_ = self.getMaxSpeedLastSeg(Blast, t)
                dqmax.append(dqmax_)

        dqmax = np.asarray(dqmax).T                     # Returns indices (np.argmax(dq_max, axis=1))
        dqmax = np.amax(dqmax, axis=1)                  # Returns values axis = 0 -> column-wise, axis=1, row-wise
                                                        # print(np.amax(dq_max, axis=1))
        return dqmax

    def ho_cook(self, cartesian, t = None, q=None, first = True):

        # List of joint positions that must be visited
        if first:
            q = [self.get_ik(arrayToPose(pos)) for pos in cartesian]
            #print("Inverse kinematics q is: {}".format(q))
            # Time parametrization
            t = self.get_time_parametrization(q)
            print("Initial parametrization is: {}".format(t))
        # Ap matrix
        Ap = self.createApmatrix(q, t)
        #print("Ap [{}] is: {}".format(Ap.shape, Ap))
        Mp = self.createMpmatrix(t)
        #print("Mp [{}] is: {}".format(Mp.shape, Mp))
        A = self.createAmatrix(q, t, Ap)
        #print("A [{}] is: {}".format(A.shape, A))
        M = self.createMmatrix(Mp, t)
        #print("M [{}] is: {}".format(M.shape, M))
        dq = np.matmul(A, np.linalg.inv(M))
        #print("dq [{}] is: {}".format(dq.shape, dq))
        zeros = np.zeros((self.n_joints, 1)); dq = np.hstack((zeros, dq)); dq = np.hstack((dq, zeros))
        #print("q len is: {}".format(len(q)))
        dq_max = self.get_dq_max(q, dq, t)
        dq_max_val = np.max(dq_max) 
        scaling_factor = dq_max_val/self.joint_max
        # Scale to accomodate limits
        t = [round(t_*scaling_factor, 5) for t_ in t]

        if scaling_factor > 1.0:
            print("Scaling factor is: {}".format(scaling_factor))
            return self.ho_cook(cartesian, t, q, first=False)

        else:
            trajectory = self.createTrajectory(q, dq, t)
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
        points = self.taylor_interpolate_points([poseToArray(self.current_pose), poseToArray(goal_pose)], eps)
        # TODO: Normalize quaternion before hocook 
        # Add time parametrization
        exec_duration = self.ho_cook(points)
        rospy.sleep(exec_duration)
        #draw(self.ee_points, self.ee_points_fk, points, eps)
        return points
        
    def go_to_start_pose(self, start_pose):
        q_start = self.get_ik(start_pose)
        self.execute_cmd(q_start)

    def go_to_pose_taylor(self, goal_pose, eps):
        # Reset saved ee_points and fk points
        self.ee_points = []; self.ee_points_fk = []
        # Start time counting
        start_time = rospy.Time.now().to_sec()
        # Calculate taylor points
        points = self.taylor_interpolate_points([poseToArray(self.current_pose), poseToArray(goal_pose)], eps)
        # get ik_poses from calculated_taylor_points (remove first point)
        ik_poses = [self.get_ik(arrayToPose(pose)) for pose in points[1:]]
        duration = rospy.Time.now().to_sec() - start_time
        # Info print
        rospy.loginfo("Number of points is: {}".format(len(points)))
        rospy.loginfo("Taylor interpolation duration is: {}".format(duration))
        # Execute q_cmds
        self.execute_cmds(ik_poses)
        return points
    
    def reset_taylor(self): 
        self.taylor_points = []

    def run(self):
        # Initialize starting pose
        self.pose_pub.publish(self.pose_A)
        # Sleep for 5. seconds
        rospy.sleep(10.0)

        eps_ = [0.03, 0.015, 0.01]
        while not rospy.is_shutdown():
            for e in eps_:
                self.pose_pub.publish(self.pose_A)
                rospy.sleep(5)
                # Go to point B
                points = self.go_to_pose_ho_cook(self.poses[4], e)
                draw(self.ee_points, self.ee_points_fk, points, e)
                self.reset_taylor()
                # Go to point C 
                rospy.sleep(2)
                #points = self.go_to_pose_ho_cook(self.pose_C, e)
                #draw(self.ee_points, self.ee_points_fk, points, e)
                #points.append(self.go_to_pose_ho_cook(self.pose_C, e))
            break

        exit()


if __name__ == "__main__":
    lab2 = OrLab2()
    lab2.run()