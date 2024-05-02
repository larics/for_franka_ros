#!/usr/bin/env python3

import rospy
import copy
import math
import numpy as np
import csv
from numpy import linalg
from geometry_msgs.msg import Pose
from tf import TransformListener, LookupException, ConnectivityException, ExtrapolationException
from std_msgs.msg import Bool
from kalip_utils import *
from sensor_msgs.msg import Joy, PointCloud



def euclidean_distance_between_poses(pose1 : Pose, pose2: Pose):
    pose1_position = pose1.position
    pose2_position = pose2.position
    position1 = [pose1_position.x, pose1_position.y, pose1_position.z]
    position2 = [pose2_position.x, pose2_position.y, pose2_position.z]
    return np.linalg.norm(np.array(position1) - np.array(position2))

class sendToKalipPose():
    def __init__(self):
        rospy.init_node("send_to_kalip_node", anonymous=True, log_level=rospy.INFO)
        self.kalip_pose = Pose()
        self.cmd_pose = Pose()
        self.msg_reciv = False
        self.msg_reciv_one_pose = False
        self.msg_reciv_list_pose = False
        self.pose_sent = False
        #Peak tool on top of franka hand
        self.T_EF_PEAK = np.eye(4)
        p_EF_PEAK = [-0.0009063,  -0.00021616,  0.0637572]
        #T_EF_PEAK[0,3] = p_EF_PEAK[0]
        #T_EF_PEAK[1,3] = p_EF_PEAK[1]
        self.T_EF_PEAK[2,3] = p_EF_PEAK[2]
        
        self.kalip_pose_off = Pose()
        
        # Counter for sending it to list 
        self.cnt = 0; 
        self.rate = 0.2; 
        self.cmd_pose_list = []
        self.current_robot_pose = Pose()
        self.achieved_robot_position_list = []
        self._init_subs()
        self._init_pubs()
        self.click = False
        #CSV WRITER FOR MARKED AND ACHIEVED ROBOT POSITIONS (Kalipen position, Robot,position)
        self.csv_file1 = open('marked_and_robot_positions.csv', 'w')  # Open CSV file in write mode
        self.csv_writer1 = csv.writer(self.csv_file1)
        self.csv_writer1.writerow(['x_marked', 'y_marked', 'z_marked', 'x_robot','y_robot','z_robot'])  # Write header
        #CSV WRITER FOR MARKING POSITIONS WITH ROBOT
        self.csv_file2 = open('robot_end_effector_positions_of_peak.csv', 'w')  # Open CSV file in write mode
        self.csv_writer2 = csv.writer(self.csv_file2)
        self.csv_writer2.writerow(['x_EF_robot','y_EF_robot','z_EF_robot'])  # Write header
        #CSV WRITER FOR ROBOT EF POSE
        self.csv_file3 = open('robot_EF_poses.csv', 'w')  # Open CSV file in write mode
        self.csv_writer3 = csv.writer(self.csv_file3)
        self.csv_writer3.writerow(['x', 'y', 'z', 'qx','qy','qz','qw'])  # Write header
        #CSV WRITER FOR Peak Position and Kalipen Position
        self.csv_file4 = open('Kalipen_positions_and_robot_peak_position.csv', 'w')  # Open CSV file in write mode
        self.csv_writer4 = csv.writer(self.csv_file4)
        self.csv_writer4.writerow(['x_robot', 'y_robot', 'z_robot', 'x_kaliepn','y_kalipen','z_kalipen'])  # Write header   
        
        self.csv_file5 = open('robot_EFF_position.csv', 'w')  # Open CSV file in write mode
        self.csv_writer5 = csv.writer(self.csv_file5)
        self.csv_writer5.writerow(['x_robot', 'y_robot', 'z_robot'])  # Write header 
        
        self.csv_file6 = open('Kalip_position.csv', 'w')  # Open CSV file in write mode
        self.csv_writer6 = csv.writer(self.csv_file6)
        self.csv_writer6.writerow(['x_kalip', 'y_kalip', 'z_kalip'])  # Write header       
        

    def _init_pubs(self):
        self.pose_pub = rospy.Publisher("/control_arm_node/arm/command/cmd_pose", Pose, queue_size=10, latch=True)
        rospy.loginfo("Initialized publishers!")

    def _init_subs(self):
        self.pose_sub = rospy.Subscriber("/Kalipen/pose_transformed_base_frame", Pose, self.kalip_cb, queue_size=1)
        # TODO: Capture and send --> If capture and send, capture current pose and send it there 
        # TODO: Just capture --> If just capture, append poses to the list and then send it taking in consideration list ordering
        # TODO: Just send --> Just send on the last captured pose
        self.kalip_curr_sub = rospy.Subscriber("/skp/capture_and_send_to_kalip", Bool, self.capture_and_send_to_kalip_cb, queue_size=1)
        self.kalip_capture_sub = rospy.Subscriber("/skp/capture_kalip_pose", Bool, self.capture_kalip_cb, queue_size=1)
        self.send_sub = rospy.Subscriber("/skp/send_to_captured_pose", Bool, self.send_to_captured_kalip_cb, queue_size=1)
        
        self.send_list = rospy.Subscriber("/skp/send_through_list_of_poses", Bool, self.send_through_list, queue_size=1)#callback
        self.sub_curent_robot_pose = rospy.Subscriber("/control_arm_node/arm/state/current_pose", Pose, self.curr_pose_cb, queue_size=1)#CALLBACK
        rospy.Subscriber('/kalipen/joy', Joy, self.click_callback, queue_size=1)

        
        rospy.loginfo("Initialized subscribers!") 

    ## Callbacks
    def click_callback(self, click: Joy):
        if (click.buttons[1] == 1):
            rospy.loginfo("Capturing kalipen pose")
            self.cmd_pose.position = self.kalip_pose_off.position
            self.cmd_pose.orientation = self.kalip_pose_off.orientation
            self.cmd_pose_list.append(self.kalip_pose_off)
            print(self.cmd_pose_list)
        if (click.buttons[2] == 1):
            rospy.loginfo("Sending arm through list of poses")
            self.msg_reciv_list_pose = True
            
        if (click.buttons[3] == 1):
            rospy.loginfo("Capturing Robot END EFFECTOR Position")

            T_BASE_EF = pose_to_T(self.current_robot_pose)
            
            T_BASE_PEAK = T_BASE_EF @  self.T_EF_PEAK
            
            self.csv_writer2.writerow([T_BASE_PEAK[0,3], T_BASE_PEAK[1,3], T_BASE_PEAK[2,3]])
            self.csv_writer3.writerow([self.current_robot_pose.position.x, self.current_robot_pose.position.y, self.current_robot_pose.position.z,
                                       self.current_robot_pose.orientation.x,self.current_robot_pose.orientation.y,
                                       self.current_robot_pose.orientation.z,self.current_robot_pose.orientation.w])
            
            self.csv_writer4.writerow([T_BASE_PEAK[0,3], T_BASE_PEAK[1,3], T_BASE_PEAK[2,3], self.kalip_pose.position.x, self.kalip_pose.position.y, self.kalip_pose.position.z])      

        if (click.buttons[4] == 1): #L1 - robot EEF position
            T_BASE_EF = pose_to_T(self.current_robot_pose)
            
            T_BASE_PEAK = T_BASE_EF @  self.T_EF_PEAK
            self.csv_writer5.writerow([T_BASE_PEAK[0,3], T_BASE_PEAK[1,3], T_BASE_PEAK[2,3]])
            
        if (click.buttons[5] == 1): #R1 - Kalip position
             self.csv_writer6.writerow([self.kalip_pose.position.x, self.kalip_pose.position.y, self.kalip_pose.position.z])
            
    def kalip_cb(self, msg):
        # TODO: Add checks for the Kalipen acq position
        self.kalip_pose.position = msg.position
        self.kalip_pose.orientation = msg.orientation 
        # Add hardcoding of the orientation for the first test
        #qx = 0.68285; qy = 0.017823; qz = 0.73026; qw = 0.010886;
        #normq = np.sqrt(qx**2 + qy**2+qz**2+qw**2) 
        #self.kalip_pose.orientation.x = qx/normq
        #self.kalip_pose.orientation.y = qy/normq
        #self.kalip_pose.orientation.z = qz/normq
        #self.kalip_pose.orientation.w = qw/normq
        # Adding EE tool offset correction
        #self.kalip_pose_off = addOffsetToPose(self.kalip_pose, 0.0, -0.009)
        
        self.kalip_pose_off = T_to_pose(pose_to_T(self.kalip_pose) @ linalg.inv(self.T_EF_PEAK) )
        #print(self.kalip_pose_off)
    def capture_and_send_to_kalip_cb(self, msg):
        if msg.data == True:
            rospy.loginfo("Capturing kalipen and sending arm!")
            self.cmd_pose.position = self.kalip_pose_off.position
            self.cmd_pose.orientation = self.kalip_pose_off.orientation
            self.msg_reciv = True;  

    def capture_kalip_cb(self, msg):
        if msg.data == True:
            rospy.loginfo("Capturing kalipen pose")
            self.cmd_pose.position = self.kalip_pose_off.position
            self.cmd_pose.orientation = self.kalip_pose_off.orientation
            self.cmd_pose_list.append(self.kalip_pose_off)
            print(self.cmd_pose_list)

    def send_to_captured_kalip_cb(self, msg):
        if msg.data == True:
            rospy.loginfo("Sending arm to captured Kalipen pose!")
            self.msg_reciv_one_pose = True

    def sendRobotToPose(self, pose, sleepT=5.0):
        self.pose_pub.publish(pose)
        rospy.sleep(sleepT)
        
    def curr_pose_cb(self, msg):
        #rospy.loginfo("Reciv pose cb {}".format(self.current_robot_pose))
        self.current_robot_pose.position = msg.position
        self.current_robot_pose.orientation = msg.orientation
        
    def send_through_list(self, msg):
        if msg.data == True:
            rospy.loginfo("Sending arm through list of poses")
            self.msg_reciv_list_pose = True
        
    def run(self):
        eps = 0.005
        if self.msg_reciv_one_pose:
            self.sendRobotToPose(self.cmd_pose)
            self.msg_reciv_one_pose = False
            rospy.sleep(self.rate)
        elif self.msg_reciv_list_pose:
            if not self.pose_sent: 
                self.pose_pub.publish(self.cmd_pose_list[self.cnt])
                self.pose_sent = True                
            # Calculate distance
            dist = euclidean_distance_between_poses(self.cmd_pose_list[self.cnt], self.current_robot_pose)
            if  dist > eps: 
                rospy.loginfo("Travelling!") 
            else: 
                rospy.loginfo("Distance: {}".format(dist)) 
                self.pose_sent = False
                # TODO: Saved achieved position list
                self.csv_writer1.writerow([self.cmd_pose_list[self.cnt].position.x,self.cmd_pose_list[self.cnt].position.y, self.cmd_pose_list[self.cnt].position.z, 
                                           self.current_robot_pose.position.x, self.current_robot_pose.position.y, self.current_robot_pose.position.z])

                self.achieved_robot_position_list.append(self.current_robot_pose.position)
                self.cnt = len(self.achieved_robot_position_list)
                if self.cnt == len(self.cmd_pose_list):
                    self.msg_reciv_list_pose = False
                    rospy.loginfo("Visited points: {}".format(self.achieved_robot_position_list))
                
        rospy.sleep(self.rate)
                

if __name__ == "__main__":
    sKP = sendToKalipPose()
    while not rospy.is_shutdown():
        try:
            sKP.run()
        except KeyboardInterrupt: 
            break