#!/bin/bash

# Set the path where you want to save the recorded bag file
output_path="."
bag_filename="recorded_data.bag"

# Specify the ROS topics to record
topics="/joint_states /position_joint_trajectory_controller/command"

# Start recording using rosbag
rosbag record -O "$output_path/$bag_filename" $topics

