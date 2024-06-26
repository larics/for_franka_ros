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

        # Initialize joy and cmd_vel variables
        self.joyData = Joy()
        self.armPoseCurr = Pose() 
        self.enable = False
        self.reciv_pose = False
        self.ready = False
        self.servo = True
        self.topic_timeout = 1

        # Subscriber to joystick topic
        self.joySub = rospy.Subscriber("/joy", Joy, self.joyCallback, queue_size=1)

        if self.servo: 
            servo_ns = "control_arm_servo_node"
            self.armPosePub = rospy.Publisher(f"/{servo_ns}/target_pose", PoseStamped, queue_size=1)
            self.currPoseSub = rospy.Subscriber(f"{servo_ns}/arm/state/current_pose", Pose, self.poseCallback, queue_size=1)
        else: 
            n_servo_ns = "control_arm_node"
            self.armPosePub = rospy.Publisher(f"/{n_servo_ns}/arm/command/cmd_pose", Pose, queue_size=1)
            self.currPoseSub = rospy.Subscriber(f"{n_servo_ns}/arm/state/current_pose", Pose, self.poseCallback, queue_size=1)

        # Resolution --> set to be increasable by joystick
        self.scaleX = 0.004
        self.scaleY = 0.004
        self.scaleZ = 0.004

    def run(self):
        r = rospy.Rate(100)
        while not rospy.is_shutdown():
            start_time = rospy.Time.now().to_sec()
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
                rospy.loginfo(f"Loop duration is: {rospy.Time.now().to_sec() - start_time}")
            else:
                rospy.loginfo_throttle(5.0, "[ArmJoyCtl] Off")
            
            r.sleep()

    def create_arm_cmd(self): 
        if self.servo: 
            cmd = PoseStamped()
            cmd.header.stamp = rospy.Time.now()
            cmd.header.frame_id = "panda_link0"
            # Translation
            cmd.pose.position.x = self.armPoseCurr.position.x + self.dX * self.scaleX
            cmd.pose.position.y = self.armPoseCurr.position.y + self.dY * self.scaleY
            cmd.pose.position.z = self.armPoseCurr.position.z + self.dZ * self.scaleZ
            # Rotation --> postpone, test this first
            cmd.pose.orientation = self.armPoseCurr.orientation
        else:
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