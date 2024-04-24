#!/bin/bash

# Set the path where you want to save the recorded bag file
output_path="."
bag_filename="servo_franka_data.bag"

# Specify the ROS topics to record
# topics="/joint_states /position_joint_trajectory_controller/command /p2p/joint_states /hocook/joint_states /taylor/joint_states /control_arm_node/tool/current_pose"
TOPICS="/joint_states /control_arm_servo_node/target_pose /control_arm_servo_node/delta_twist_cmds /control_arm_servo_node/arm/state/current_pose"


# Start recording using rosbag
rosbag record -O "$output_path/$bag_filename" $TOPICS

