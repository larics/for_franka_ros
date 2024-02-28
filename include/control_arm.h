#ifndef CONTROL_ARM_H
#define CONTROL_ARM_H

#include <chrono>
#include <cmath>
#include <iostream>
#include <pthread.h>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

#include <yaml-cpp/yaml.h>

// ROS
#include <controller_manager_msgs/ListControllers.h>
#include <controller_manager_msgs/SwitchController.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/TransformStamped.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_srvs/Trigger.h>
#include <std_srvs/TriggerRequest.h>
#include <std_srvs/TriggerResponse.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// MoveIt
#include <moveit/collision_detection/collision_matrix.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/GetPositionIKRequest.h>
#include <moveit_msgs/GetPositionIKResponse.h>
#include <moveit_msgs/PositionIKRequest.h>

// Conversions
#include <eigen_conversions/eigen_msg.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>

// Utils
#include <tf2/LinearMath/Quaternion.h>

// Custom service
#include "for_franka_ros/getIk.h"
#include "for_franka_ros/getIkRequest.h"
#include "for_franka_ros/getIkResponse.h"


class ControlArm {

public:
  // Constructor
  ControlArm(ros::NodeHandle nh);

  // Destructor
  ~ControlArm();

  // Variables
  geometry_msgs::Point currentEEPosition;
  geometry_msgs::Pose currentEEPose;

  // Setters
  bool setCmdPose();
  bool setMoveGroup();
  bool setPlanningScene();

  // Getters
  void getBasicInfo();

  // Pointer to move group
  // (https://answers.ros.org/question/344598/cant-create-movegroupinterface-object-in-my-own-class/)
  moveit::planning_interface::MoveGroupInterface *m_moveGroupPtr;
  const robot_state::JointModelGroup *m_jointModelGroupPtr;
  planning_scene::PlanningScene *m_planningScenePtr;
  moveit::core::RobotStatePtr m_currentRobotStatePtr;
  robot_model_loader::RobotModelLoader m_robotModelLoader;
  Eigen::Affine3d m_endEffectorState;

  // Run method
  void run();

  // It's not possible to access private members of this class from another class
  // Link to issue that explains it:
  // https://stackoverflow.com/questions/18944451/how-to-make-a-derived-class-access-the-private-member-data
private:

  // Initialization method
  void initRobot();

  // ROS node handle
  ros::NodeHandle nH;
  ros::NodeHandle nHns;

  // transformListener
  tf::TransformListener listener;
  tf::TransformBroadcaster broadcaster;

  // Basic robot setup
  std::string GROUP_NAME; 
  std::string EE_LINK_NAME; 

  // Topic names
  std::string dispTrajTopicName; 
  std::string currPoseTopicName;
  std::string cmdPoseTopicName;
  std::string cmdToolOrientTopicName;
  std::string cmdDeltaPoseTopicName;

  // Service names
  std::string disableCollisionSrvName;
  std::string addCollisionObjectSrvName;
  std::string startPositionCtlSrvName;
  std::string startJointTrajCtlSrvName;
  std::string startJointGroupPosCtlSrvName;
  std::string startJointGroupVelCtlSrvName;
  std::string getIkSrvName; 

  // Topic queue sizes
  int dispTrajQSize;
  int currPoseQSize;
  int cmdPoseQSize;
  int cmdToolOrientQSize;
  int cmdDeltaPoseQSize;

  // ROS Publishers
  ros::Publisher dispTrajPub;
  ros::Publisher currPosePub;
  ros::Publisher cmdQ1Pub;
  ros::Publisher cmdJointGroupPositionPub;
  ros::Publisher cmdJointGroupVelocityPub;

  // ROS Subscribers
  ros::Subscriber cmdPoseSub;
  ros::Subscriber cmdDeltaPoseSub;
  ros::Subscriber cmdToolOrientSub;

  // ROS Services
  ros::ServiceServer getIkSrv; 
  ros::ServiceServer disableCollisionSrv;
  ros::ServiceServer addCollisionObjectSrv;
  ros::ServiceServer startPositionCtlSrv;
  ros::ServiceServer startJointTrajCtlSrv;
  ros::ServiceServer startJointGroupPositionCtlSrv;
  ros::ServiceServer startJointGroupVelocityCtlSrv;

  // ROS Service clients
  ros::ServiceClient applyPlanningSceneSrvCli;
  ros::ServiceClient realRobotDriverInitSrvCli;
  ros::ServiceClient addCollisionObjectSrvCli;
  ros::ServiceClient switchCtlSrvCli;
  ros::ServiceClient listCtlSrvCli;
  ros::ServiceClient switchToPositionCtlSrvCli;
  ros::ServiceClient switchToTrajectoryCtlSrvCli;

  // ROS Subscriber Callback
  void cmdPoseCb(const geometry_msgs::Pose::ConstPtr &msg);
  void cmdDeltaPoseCb(const geometry_msgs::Pose::ConstPtr &msg);
  void cmdToolOrientationCb(const geometry_msgs::Point::ConstPtr &msg);

  // ROS Services callbacks
  bool disableCollisionSrvCb(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);
  bool addCollisionObjectSrvCb(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);
  bool startPositionCtlCb(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);
  bool startJointTrajCtlCb(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);
  bool startJointGroupPositionCtlCb(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);
  bool startJointGroupVelocityCtlCb(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);
  bool getIkSrvCb(for_franka_ros::getIkRequest &req, for_franka_ros::getIkResponse &res); 

  // DisplayTrajectory
  moveit_msgs::DisplayTrajectory displayTrajectory_;

  // Private variables
  int sleepMs_;
  bool realRobot_;
  bool enableVisualization_;
  bool moveGroupInitialized_;
  bool planningSceneInitialized_;
  bool blockingMovement = true;
  std::string endEffectorLinkName;
  geometry_msgs::Pose m_cmdPose;
  geometry_msgs::Pose m_lastCmdPose;
  geometry_msgs::Pose m_cmdDeltaPose;

  // Vectors and arrays
  std::vector<double> m_jointPositions_;

  bool sendToCmdPose();
  void sendToCmdPoses(std::vector<geometry_msgs::Pose> poses);
  bool sendToDeltaCmdPose();
  void addCollisionObject(moveit_msgs::PlanningScene &planningScene);
  void getCurrentArmState();
  void getCurrentEndEffectorState(const std::string linkName);
  void getJointPositions(const std::vector<std::string> &jointNames,
                         std::vector<double> &jointGroupPositions);
  void getRunningControllers(std::vector<std::string> &runningControllerNames);
  bool getIK(const geometry_msgs::Pose wantedPose, const std::size_t attempts, double timeout);
  bool getAnalyticIK(const geometry_msgs::Pose wantedPose); 

  Eigen::MatrixXd getJacobian(Eigen::Vector3d
                  refPointPosition); // Can be created as void and arg passed to
                                     // be changed during execution
  Eigen::MatrixXd getInertiaMatrix(Eigen::Vector3d refPointPosition);

  // TODO: Move this to utils.cpp
  float round(float var);

  

};

#endif // CONTROL_ARM_H