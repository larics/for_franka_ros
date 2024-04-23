#!/usr/bin/env python3
# _author_: Filip Zorić; filip.zoric@fer.hr


import rospy
from geometry_msgs.msg import Twist, Pose, PoseStamped, TwistStamped
from sensor_msgs.msg import Joy

# TODO: 
# - [] Add flag for the orientation handling 
# - [] Add orientation control 
# - [] Test joy control node without trajectory planning 

class ArmJoy:

    def __init__(self):
        # Publisher to ardrone cmd_vel topic, can be run in namespace
        self.armPosePub = rospy.Publisher("/control_arm_node/arm/command/cmd_pose", Pose, queue_size=1)

        # Initialize joy and cmd_vel variables
        self.joyData = Joy()
        self.armPoseCurr = Pose() # Publishing to real cmd_vel
        self.enable = False
        self.reciv_pose = False
        self.ready = False
        self.topic_timeout = 1

        # Subscriber to joystick topic
        self.joySub = rospy.Subscriber("/joy", Joy, self.joyCallback, queue_size=1)
        self.currPoseSub = rospy.Subscriber("/control_arm_node/arm/state/current_pose", Pose, self.poseCallback, queue_size=1)

        # Resolution --> set to be increasable by joystick
        self.scaleX = 0.01
        self.scaleY = 0.01
        self.scaleZ = 0.01

    def run(self):
        r = rospy.Rate(50)
        while not rospy.is_shutdown():
            if self.reciv_pose: 
                self.ready = True
                # Simple timeout condition -> prevent from fetching wrong joy pose
                if rospy.Time.now().to_sec() - self.last_reciv_t < 1: 
                    self.reciv_pose = False
            else: 
                self.ready = False
            if self.enable and self.ready:
                armPoseCmd = self.create_arm_cmd()

                # TODO: Test for the rotation purposes
                rospy.loginfo_throttle(1.0, "[ArmJoyCtl] On")
                self.armPosePub.publish(armPoseCmd)
            else:
                rospy.loginfo_throttle(5.0, "[ArmJoyCtl] Off")
            r.sleep()

    def create_arm_cmd(self): 
        cmd = Pose()
        # Translation
        cmd.position.x = self.armPoseCurr.position.x + self.dX * self.scaleX
        cmd.position.y = self.armPoseCurr.position.y + self.dY * self.scaleY
        cmd.position.z = self.armPoseCurr.position.z + self.dZ * self.scaleZ
        # Rotation --> postpone, test this first
        cmd.orientation = self.armPoseCurr.orientation
        return cmd

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
        self.reciv_pose = True
        self.armPoseCurr.position.x = msg.position.x
        self.armPoseCurr.position.y = msg.position.y
        self.armPoseCurr.position.z = msg.position.z
        self.armPoseCurr.orientation.x = msg.orientation.x
        self.armPoseCurr.orientation.y = msg.orientation.y
        self.armPoseCurr.orientation.z = msg.orientation.z
        self.armPoseCurr.orientation.w = msg.orientation.w
        self.last_reciv_t = rospy.Time.now().to_sec()

if __name__ == "__main__":
    rospy.init_node("ArmJoyNode")
    joyCtl = ArmJoy()
    joyCtl.run()

# Enable: R2