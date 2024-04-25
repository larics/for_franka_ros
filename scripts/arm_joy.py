#!/usr/bin/env python3
# _author_: Filip Zorić; filip.zoric@fer.hr


import rospy
from geometry_msgs.msg import Twist, Pose, PoseStamped, TwistStamped
from sensor_msgs.msg import Joy
from std_msgs.msg import String
from for_franka_ros.srv import changeState

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
        # Ready to send servo cmds to arm
        self.ready = False
        # Servo flag
        self.servo = True
        # Change state of the arm to enable servo
        self.change_state = False
        self.topic_timeout = 1
        self.state_cnt = 0
        self.curr_state = "IDLE"
        self.states = ["JOINT_TRAJ_CTL", "CART_TRAJ_CTL", "SERVO_CTL", "IDLE"]

        # Subscriber to joystick topic
        self.joySub = rospy.Subscriber("/joy", Joy, self.joyCallback, queue_size=1)

        # TODO: merge those two when required
        if self.servo: 
            servo_ns = "control_arm_servo_node"
            self.armPosePub = rospy.Publisher(f"/{servo_ns}/target_pose", PoseStamped, queue_size=1)
            self.armVelPub = rospy.Publisher(f"/{servo_ns}/delta_twist_cmds", TwistStamped, queue_size=1)
            self.currPoseSub = rospy.Subscriber(f"{servo_ns}/arm/state/current_pose", Pose, self.poseCallback, queue_size=1)
            # TODO: Add changing robot state as a service to the joystick node
            self.stateProxy = rospy.ServiceProxy(f"{servo_ns}/change_state", changeState)
        else: 
            n_servo_ns = "control_arm_node"
            self.armPosePub = rospy.Publisher(f"/{n_servo_ns}/arm/command/cmd_pose", Pose, queue_size=1)
            self.currPoseSub = rospy.Subscriber(f"{n_servo_ns}/arm/state/current_pose", Pose, self.poseCallback, queue_size=1)

        # Resolution --> set to be increasable by joystick
        self.scaleX = 0.01 # m/s
        self.scaleY = 0.01 # m/s
        self.scaleZ = 0.01 # m/s

    def run(self):
        r = rospy.Rate(5)
        while not rospy.is_shutdown():
            if self.reciv_pose: 
                self.ready = True
                # Simple timeout condition -> prevent from fetching wrong joy pose
                if rospy.Time.now().to_sec() - self.last_reciv_t > 1: 
                    self.reciv_pose = False
                    rospy.logwarn("Not recieving arm pose anymore!")
            else: 
                self.ready = False

            # Recieved relevant pose, and pressed enable
            if self.ready and self.enable:
                if self.change_state:
                    # Change state
                    resp = self.stateProxy(str(self.states[self.state_cnt]))
                    self.armPoseCmd = self.create_arm_cmd()
                    self.change_state = False
                    self.curr_state = self.states[self.state_cnt] 
                # Use changed state to publish new joy msg
                if self.curr_state == "SERVO_CTL":
                    self.armPoseCmd = self.create_arm_cmd()
                    armVelCmd = self.create_vel_cmd()
                    self.armVelPub.publish(armVelCmd)
                    #self.armPosePub.publish(self.armPoseCmd)
                rospy.loginfo_throttle(5.0, "[ArmJoyCtl] On")
            else:
                rospy.loginfo_throttle(5.0, "[ArmJoyCtl] Off")
            
            r.sleep()

    
    def create_arm_cmd(self): 
        if self.servo: 
            cmd = PoseStamped()
            cmd.header.stamp = rospy.Time.now()
            cmd.header.frame_id = "panda_hand_tcp"
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
    
    def create_vel_cmd(self): 
        cmd = TwistStamped()
        cmd.header.stamp = rospy.Time.now()
        cmd.header.frame_id = "panda_hand_tcp"
        cmd.twist.linear.x = self.dX * self.scaleX
        cmd.twist.linear.y = self.dY * self.scaleY
        cmd.twist.linear.z = self.dZ * self.scaleZ
        return cmd

    def joyCallback(self, data):
        # Assign data to joy variable
        self.joyData = data
        # Setting joy values to be command values for bebop
        self.dX = self.joyData.axes[3]
        self.dY = self.joyData.axes[2]
        self.dZ = self.joyData.axes[1]
        self.dYaw = self.joyData.axes[0]
        # Removed from condition
        if self.joyData.buttons[7] == 1: 
            self.enable = True
            rospy.loginfo("Joystick is enabled!")
        if self.joyData.buttons[5] == 1: 
            self.enable = False
            rospy.loginfo("Joystick is disabled!")

        if self.joyData.axes[-1] == 1:
            if self.state_cnt < 3: 
                self.state_cnt += 1
            else: 
                self.state_cnt = 0 
            rospy.loginfo(f"Choosen state is: {self.states[self.state_cnt]}")

        if self.joyData.buttons[6] == 1: 
            self.change_state = True

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