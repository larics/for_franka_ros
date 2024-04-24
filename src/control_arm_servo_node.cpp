#include <ros/ros.h>
#include "control_arm_servo.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "control_arm_servo");
    ros::NodeHandle nodeHandle("~"); 
    ControlArmServo controlArmServo(nodeHandle); 

    int num_threads = 6;     
    ros::AsyncSpinner spinner(num_threads);
    spinner.start(); 
    controlArmServo.run(); 
    spinner.stop(); 

    return 0; 

}