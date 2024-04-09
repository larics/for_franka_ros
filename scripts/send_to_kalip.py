#!/usr/bin/env python3

import rospy
import copy
import math
import numpy as np

from geometry_msgs.msg import Pose
from tf import TransformListener, LookupException, ConnectivityException, ExtrapolationException
from std_msgs.msg import Bool
from kalip_utils import *

class sendToKalipPose():
    def __init__(self):
        rospy.init_node("send_to_kalip_node", anonymous=True, log_level=rospy.INFO)
        self.kalip_pose = Pose()
        self.cmd_pose = Pose()
        self.msg_reciv = False
        self.rate = 1; 
        self._init_subs()
        self._init_pubs()

    def _init_pubs(self):
        self.pose_pub = rospy.Publisher("/control_arm_node/arm/command/cmd_pose", Pose, queue_size=10, latch=True)
        rospy.loginfo("Initialized publishers!")

    def _init_subs(self):
        self.pose_sub = rospy.Subscriber("/Kalipen/pose_transformed_base_frame", Pose, self.kalip_cb, queue_size=1)
        # TODO: Capture and send --> If capture and send, capture current pose and send it there 
        # TODO: Just capture --> If just capture, append poses to the list and then send it taking in consideration list ordering
        # TODO: Just send --> Just send on the first captured pose
        self.kalip_curr_sub = rospy.Subscriber("/skp/capture_and_send_to_kalip", Bool, self.capture_and_send_to_kalip_cb, queue_size=1)
        self.kalip_capture_sub = rospy.Subscriber("/skp/capture_kalip_pose", Bool, self.capture_kalip_cb, queue_size=1)
        self.send_sub = rospy.Subscriber("/skp/send_to_captured_pose", Bool, self.send_to_captured_kalip_cb, queue_size=1)
        rospy.loginfo("Initialized subscribers!") 

    ## Callbacks
    def kalip_cb(self, msg):
        # TODO: Add checks for the Kalipen acq position
        self.kalip_pose.position = msg.position
        self.kalip_pose.orientation = msg.orientation
        # Adding EE tool offset correction
        self.kalip_pose = addOffsetToPose(self.kalip_pose, -0.045)

    def capture_and_send_to_kalip_cb(self, msg):
        if msg.data == True:
            rospy.loginfo("Capturing kalipen and sending arm!")
            self.cmd_pose.position = self.kalip_pose.position
            self.cmd_pose.orientation = self.kalip_pose.orientation
            self.msg_reciv = True;  

    def capture_kalip_cb(self, msg):
        if msg.data == True:
            rospy.loginfo("Capturing kalipen pose")
            self.cmd_pose.position = self.kalip_pose.position
            self.cmd_pose.orientation = self.kalip_pose.orientation

    def send_to_captured_kalip_cb(self, msg):
        if msg.data == True:
            rospy.loginfo("Sending arm to captured Kalipen pose!")
            self.msg_reciv = True

    def sendRobotToPose(self, pose, sleepT=5.0):
        self.pose_pub.publish(pose)
        rospy.sleep(sleepT)

    def run(self):
        if self.msg_reciv: 
            self.sendRobotToPose(self.cmd_pose)
            self.msg_reciv = False
            rospy.sleep(self.rate)

if __name__ == "__main__":
    sKP = sendToKalipPose()
    while not rospy.is_shutdown():
        try:
            sKP.run()
        except KeyboardInterrupt: 
            break