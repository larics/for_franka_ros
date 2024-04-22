#!/usr/bin/env python3
# _author_: Filip Zorić; filip.zoric@fer.hr


import rospy
from geometry_msgs.msg import Twist, Pose, PoseStamped, TwistStamped
from sensor_msgs.msg import Joy

class ArmJoy:

    def __init__(self):
        # Publisher to ardrone cmd_vel topic, can be run in namespace
        self.armPosePub = rospy.Publisher("/control_arm_node/arm/command/cmd_pose", Pose, queue_size=1)

        # Initialize joy and cmd_vel variables
        self.joyData = Joy()
        self.armPoseCmd = Pose() # Publishing to real cmd_vel
        self.armPoseCurr = Pose() # Publishing to real cmd_vel
        self.enable = False

        # Subscriber to joystick topic
        self.joySub = rospy.Subscriber("/joy", Joy, self.joyCallback, queue_size=1)
        self.currPoseSub = rospy.Subscriber("/control_arm_node/arm/state/current_pose", Pose, self.poseCallback, queue_size=1)

        # Resolution
        self.scaleX = 0.0001
        self.scaleY = 0.0001
        self.scaleZ = 0.0001

    def run(self):
        r = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.enable:
                # Translation
                self.armPoseCmd.position.x = self.armPoseCurr.position.x + self.dX * self.scaleX
                self.armPoseCmd.position.y = self.armPoseCurr.position.y + self.dY * self.scaleY
                self.armPoseCmd.position.z = self.armPoseCurr.position.z + self.dZ * self.scaleZ
                # Rotation --> postpone, test this first
                # TODO: Test for the rotation purposes
                rospy.loginfo_throttle(1.0, "[ArmJoyCtl] On")
                self.armPosePub.publish(self.armPoseCmd)
            else:
                rospy.loginfo_throttle(5.0, "[ArmJoyCtl] Off")
            r.sleep()

    def joyCallback(self, data):
        # Assign data to joy variable
        self.joyData = data

        # Setting joy values to be command values for bebop
        self.dX = self.joyData.axes[3]
        self.dY = self.joyData.axes[2]
        self.dZ = self.joyData.axes[1]
        self.dYaw = self.joyData.axes[0]
        self.enable = self.joyData.buttons[7]

    def poseCallback(self, msg):
        rospy.loginfo("Reciv pose")
        self.armPoseCurr.position.x = msg.position.x
        self.armPoseCurr.position.x = msg.position.y
        self.armPoseCurr.position.z = msg.position.z

if __name__ == "__main__":
    rospy.init_node("ArmJoyNode")
    joyCtl = ArmJoy()
    joyCtl.run()

# Enable: R2