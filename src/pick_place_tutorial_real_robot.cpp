// ROS
#include <ros/ros.h>

// MoveIt
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>

// TF2
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// string
#include "std_msgs/String.h"
#include <sstream>

//json
#include <nlohmann/json.hpp>
#include <fstream>

#include <iostream>

//quat to rot
#include <Eigen/Core>

using namespace std;

class PickPlace {
  private:
  ros::Publisher chatter_pub;

  public:
  PickPlace(ros::NodeHandle *nh){
    
    //json
    using json = nlohmann::json;
    std::ifstream f("/home/developer/catkin_ws/src/optidraw/src/scripts/data.json");
    json data;
    f >> data;
    std::string jsonString = data;
    json jsonData = json::parse(jsonString);

    // publish string to enable/disable object storage
    chatter_pub = nh->advertise<std_msgs::String>("/chatter", 1000);
    std::stringstream ss;
    
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::WallDuration(1.0).sleep();
    // object manipulatoin
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    moveit::planning_interface::MoveGroupInterface group("panda_arm");

    group.setPlanningTime(45.0);

    // publish string
    ss.str("Active");
    publish_chatter(ss);

    addCollisionObjects(planning_scene_interface, jsonData);

    // Wait a bit for ROS things to initialize
    ros::WallDuration(1.0).sleep();

    ss.str("Active");
    publish_chatter(ss);

    pick(group);
    // publish string

    ros::WallDuration(1.0).sleep();

    place(group);

    ros::waitForShutdown();
  }

  void publish_chatter(std::stringstream& ss) 
  {  
    std_msgs::String msg;
    msg.data = ss.str();
    chatter_pub.publish(msg); 
  }
  
  void openGripper(trajectory_msgs::JointTrajectory& posture)
  {
    // BEGIN_SUB_TUTORIAL open_gripper
    /* Add both finger joints of panda robot. */
    posture.joint_names.resize(2);
    posture.joint_names[0] = "panda_finger_joint1";
    posture.joint_names[1] = "panda_finger_joint2";

    /* Set them as open, wide enough for the object to fit. */
    posture.points.resize(1);
    posture.points[0].positions.resize(2);
    posture.points[0].positions[0] = 0.04;
    posture.points[0].positions[1] = 0.04;
    posture.points[0].time_from_start = ros::Duration(0.5);
    // END_SUB_TUTORIAL
  }

  void closedGripper(trajectory_msgs::JointTrajectory& posture)
  {
    // BEGIN_SUB_TUTORIAL closed_gripper
    /* Add both finger joints of panda robot. */
    posture.joint_names.resize(2);
    posture.joint_names[0] = "panda_finger_joint1";
    posture.joint_names[1] = "panda_finger_joint2";

    /* Set them as closed. */
    posture.points.resize(1);
    posture.points[0].positions.resize(2);
    posture.points[0].positions[0] = 0.00;
    posture.points[0].positions[1] = 0.00;
    posture.points[0].time_from_start = ros::Duration(0.5);
    // END_SUB_TUTORIAL
  }

  void pick(moveit::planning_interface::MoveGroupInterface& move_group)
  {
    // BEGIN_SUB_TUTORIAL pick1
    // Create a vector of grasps to be attempted, currently only creating single grasp.
    // This is essentially useful when using a grasp generator to generate and test multiple grasps.
    std::vector<moveit_msgs::Grasp> grasps;
    grasps.resize(1);

    // Setting grasp pose
    // ++++++++++++++++++++++
    // This is the pose of panda_link8. |br|
    // From panda_link8 to the palm of the eef the distance is 0.058, the cube starts 0.01 before 5.0 (half of the length
    // of the cube). |br|
    // Therefore, the position for panda_link8 = 5 - (length of cube/2 - distance b/w panda_link8 and palm of eef - some
    // extra padding)
    grasps[0].grasp_pose.header.frame_id = "panda_link0";
    tf2::Quaternion orientation;
    orientation.setRPY(-M_PI / 2, -M_PI / 4, -M_PI / 2);
    grasps[0].grasp_pose.pose.orientation = tf2::toMsg(orientation);
    grasps[0].grasp_pose.pose.position.x = 0.415;
    grasps[0].grasp_pose.pose.position.y = 0;
    grasps[0].grasp_pose.pose.position.z = 0.5;

    // Setting pre-grasp approach
    // ++++++++++++++++++++++++++
    /* Defined with respect to frame_id */
    grasps[0].pre_grasp_approach.direction.header.frame_id = "panda_link0";
    /* Direction is set as positive x axis */
    grasps[0].pre_grasp_approach.direction.vector.x = 1.0;
    grasps[0].pre_grasp_approach.min_distance = 0.095;
    grasps[0].pre_grasp_approach.desired_distance = 0.115;

    // Setting post-grasp retreat
    // ++++++++++++++++++++++++++
    /* Defined with respect to frame_id */
    grasps[0].post_grasp_retreat.direction.header.frame_id = "panda_link0";
    /* Direction is set as positive z axis */
    grasps[0].post_grasp_retreat.direction.vector.z = 1.0;
    grasps[0].post_grasp_retreat.min_distance = 0.1;
    grasps[0].post_grasp_retreat.desired_distance = 0.25;

    // Setting posture of eef before grasp
    // +++++++++++++++++++++++++++++++++++
    openGripper(grasps[0].pre_grasp_posture);
    // END_SUB_TUTORIAL

    // BEGIN_SUB_TUTORIAL pick2
    // Setting posture of eef during grasp
    // +++++++++++++++++++++++++++++++++++
    closedGripper(grasps[0].grasp_posture);
    // END_SUB_TUTORIAL

    // BEGIN_SUB_TUTORIAL pick3
    // Set support surface as table1.
    move_group.setSupportSurfaceName("table1");
    // Call pick to pick up the object using the grasps given
    move_group.pick("object1", grasps);
    // END_SUB_TUTORIAL
  }

  void place(moveit::planning_interface::MoveGroupInterface& group)
  {
    // BEGIN_SUB_TUTORIAL place
    // TODO(@ridhwanluthra) - Calling place function may lead to "All supplied place locations failed. Retrying last
    // location in
    // verbose mode." This is a known issue and we are working on fixing it. |br|
    // Create a vector of placings to be attempted, currently only creating single place location.
    std::vector<moveit_msgs::PlaceLocation> place_location;
    place_location.resize(1);

    // Setting place location pose
    // +++++++++++++++++++++++++++
    place_location[0].place_pose.header.frame_id = "panda_link0";
    tf2::Quaternion orientation;
    orientation.setRPY(0, 0, M_PI / 2);
    place_location[0].place_pose.pose.orientation = tf2::toMsg(orientation);

    /* While placing it is the exact location of the center of the object. */
    place_location[0].place_pose.pose.position.x = 0;
    place_location[0].place_pose.pose.position.y = 0.65;
    place_location[0].place_pose.pose.position.z = 0.5;

    // Setting pre-place approach
    // ++++++++++++++++++++++++++
    /* Defined with respect to frame_id */
    place_location[0].pre_place_approach.direction.header.frame_id = "panda_link0";
    /* Direction is set as negative z axis */
    place_location[0].pre_place_approach.direction.vector.z = -1.0;
    place_location[0].pre_place_approach.min_distance = 0.095;
    place_location[0].pre_place_approach.desired_distance = 0.115;

    // Setting post-grasp retreat
    // ++++++++++++++++++++++++++
    /* Defined with respect to frame_id */
    place_location[0].post_place_retreat.direction.header.frame_id = "panda_link0";
    /* Direction is set as negative y axis */
    place_location[0].post_place_retreat.direction.vector.y = -1.0;
    place_location[0].post_place_retreat.min_distance = 0.01;
    place_location[0].post_place_retreat.desired_distance = 0.05;

    // Setting posture of eef after placing object
    // +++++++++++++++++++++++++++++++++++++++++++
    /* Similar to the pick case */
    openGripper(place_location[0].post_place_posture);

    // Set support surface as table2.
    group.setSupportSurfaceName("table2");
    // Call place to place the object using the place locations given.
    group.place("object1", place_location);
    // END_SUB_TUTORIAL
  }

  Eigen::MatrixXd mulMat(Eigen::MatrixXd mat1, Eigen::MatrixXd mat2)
  {   
      Eigen::MatrixXd rslt(mat1.rows(), mat2.cols());
      rslt.setZero();
      for (int i = 0; i < mat1.rows(); i++) {
          for (int j = 0; j < mat2.cols(); j++) {
              for (int k = 0; k < mat1.cols(); k++) {
                  rslt(i, j) = rslt(i, j) + mat1(i, k) * mat2(k, j);
              }
          }
      }
      return rslt;
  }

  void addCollisionObjects(moveit::planning_interface::PlanningSceneInterface& planning_scene_interface, nlohmann::json_abi_v3_11_3::json& jsonData)
  {
    // BEGIN_SUB_TUTORIAL table1
    //
    // Creating Environment
    // ^^^^^^^^^^^^^^^^^^^^
    // Create vector to hold 3 collision objects.
    std::vector<moveit_msgs::CollisionObject> collision_objects;
    collision_objects.resize(8);



    std::string selected_object = "table3";
    nlohmann::json_abi_v3_11_3::json dimensions = jsonData[selected_object]["dimensions"];
    std::cout << dimensions["x"] << std::endl;
    std::cout << dimensions["y"] << std::endl;
    std::cout << dimensions["z"] << std::endl;
    nlohmann::json_abi_v3_11_3::json orientation = jsonData[selected_object]["orientation"];
    std::cout << orientation["x"] << std::endl;
    std::cout << orientation["y"] << std::endl;
    std::cout << orientation["z"] << std::endl;
    std::cout << orientation["w"] << std::endl;
    nlohmann::json_abi_v3_11_3::json position = jsonData[selected_object]["position"];
    std::cout << position["x"] << std::endl;
    std::cout << position["y"] << std::endl;
    std::cout << position["z"] << std::endl;
    
    Eigen::MatrixXd combinations(5, 3);
    combinations << 1, 0, 0,
                    -1, 0, 0,
                    0, 1, 0,
                    0, -1, 0,
                    // missing positive z which would be the lid
                    0, 0, -1;
    Eigen::MatrixXd rslt(1, 3); // Define rslt outside the loop
    for  (int i = 0; i < combinations.rows(); ++i){
        Eigen::MatrixXd selector = combinations.row(i);
        int index = 0;
        for (int j = 0; j < selector.size(); j++) {
            if (selector(0, j) != 0) {
                index = j;
                break;
            }
        }
        cout << "i " << index << endl;
        Eigen::MatrixXd dim(1, 3); // Example vector
        dim << dimensions["x"], dimensions["y"], dimensions["z"];
        //quat to rot
        // Define a quaternion
        Eigen::Quaterniond q(orientation["w"], orientation["x"], orientation["y"], orientation["z"]); // (w, x, y, z)

        // Convert quaternion to rotation matrix
        /* Eigen::Matrix3d rotation_matrix(3, 3); //= q.normalized().toRotationMatrix();
        rotation_matrix << -0.23532535, 0.94383843, 0.23192885,
                            0.94384209, 0.27886655, -0.17718793,
                            -0.23191398, 0.1772074, -0.9564588;
        Eigen::Quaterniond quaternion(rotation_matrix);
        std::cout << "Quaternion: " << quaternion.coeffs().transpose() << std::endl; */

        Eigen::Matrix3d rotation_matrix = q.normalized().toRotationMatrix();
        
        // Output the rotation matrix
        std::cout << "Rotation Matrix:" << std::endl;
        std::cout << rotation_matrix << std::endl;

        Eigen::MatrixXd rot(3, 3); // Example matrix
        rot << rotation_matrix;
        cout << "selector" << selector << endl;
        cout << "rot" << rot << endl;
        cout << "rslt" << rslt << endl;

        rslt = mulMat(rot, selector.transpose()).transpose();
        
        cout << "rslt" << rslt << endl;
        /* cout << rslt.mean() << endl;
        cout << dim.mean() << endl;
        cout << rot.mean() << endl;
        cout << rslt(0, 1) << endl;
        cout << dim << endl; */

        Eigen::MatrixXd size_outer(1, 3);
        size_outer << dim;
        
        cout << dim(0,index) << endl;
        cout << index << endl;

        Eigen::MatrixXd centroid(1, 3);
        centroid << position["x"], position["y"], position["z"];
        
        Eigen::MatrixXd tf(1, 3);
        tf << rslt * dim(0,index)/2 + centroid;
        cout << "tf" << tf << endl;
        cout << "centroid" << centroid << endl;


        size_outer(0,index) = dim.mean() * 0.01;

        rslt.setZero();
        cout << "rslt" << rslt << endl;

        // BEGIN_SUB_TUTORIAL table2
        // Add the second table where we will be placing the cube.
        std::ostringstream oss;
        oss << selected_object << i + 1;
        collision_objects[i + 2].id = oss.str();
        collision_objects[i+2].header.frame_id = "panda_link0";

        /* Define the primitive and its dimensions. */
        collision_objects[i+2].primitives.resize(1);
        collision_objects[i+2].primitives[0].type = collision_objects[i+2].primitives[0].BOX;
        collision_objects[i+2].primitives[0].dimensions.resize(3);
        collision_objects[i+2].primitives[0].dimensions[0] = size_outer(0,0);
        collision_objects[i+2].primitives[0].dimensions[1] = size_outer(0,1);
        collision_objects[i+2].primitives[0].dimensions[2] = size_outer(0,2);

        /* Define the pose of the table. */
        collision_objects[i+2].primitive_poses.resize(1);
        collision_objects[i+2].primitive_poses[0].position.x = tf(0,0);
        collision_objects[i+2].primitive_poses[0].position.y = tf(0,1);
        collision_objects[i+2].primitive_poses[0].position.z = tf(0,2) - 0.0;

        /* collision_objects[i+2].primitive_poses[0].orientation.x = quaternion.x();
        collision_objects[i+2].primitive_poses[0].orientation.y = quaternion.y();
        collision_objects[i+2].primitive_poses[0].orientation.z = quaternion.z();
        collision_objects[i+2].primitive_poses[0].orientation.w = quaternion.w(); */
 
        collision_objects[i+2].primitive_poses[0].orientation.x = orientation["x"];
        collision_objects[i+2].primitive_poses[0].orientation.y = orientation["y"];
        collision_objects[i+2].primitive_poses[0].orientation.z = orientation["z"];
        collision_objects[i+2].primitive_poses[0].orientation.w = orientation["w"];
        // END_SUB_TUTORIAL

        collision_objects[i+2].operation = collision_objects[i+2].ADD;


        }
    

    // Add the first table where the cube will originally be kept.
    collision_objects[0].id = "table1";
    collision_objects[0].header.frame_id = "panda_link0";

    /* Define the primitive and its dimensions. */
    collision_objects[0].primitives.resize(1);
    collision_objects[0].primitives[0].type = collision_objects[0].primitives[0].BOX;
    collision_objects[0].primitives[0].dimensions.resize(3);
    collision_objects[0].primitives[0].dimensions[0] = 0.2;
    collision_objects[0].primitives[0].dimensions[1] = 0.4;
    collision_objects[0].primitives[0].dimensions[2] = 0.4;

    /* Define the pose of the table. */
    collision_objects[0].primitive_poses.resize(1);
    collision_objects[0].primitive_poses[0].position.x = 0.5;
    collision_objects[0].primitive_poses[0].position.y = 0;
    collision_objects[0].primitive_poses[0].position.z = 0.2;
    // END_SUB_TUTORIAL

    collision_objects[0].operation = collision_objects[0].ADD;

    // BEGIN_SUB_TUTORIAL table2
    // Add the second table where we will be placing the cube.
    collision_objects[1].id = "table2";
    collision_objects[1].header.frame_id = "panda_link0";

    /* Define the primitive and its dimensions. */
    collision_objects[1].primitives.resize(1);
    collision_objects[1].primitives[0].type = collision_objects[1].primitives[0].BOX;
    collision_objects[1].primitives[0].dimensions.resize(3);
    collision_objects[1].primitives[0].dimensions[0] = 0.6;
    collision_objects[1].primitives[0].dimensions[1] = 0.6;
    collision_objects[1].primitives[0].dimensions[2] = 0.1;

    /* Define the pose of the table. */
    collision_objects[1].primitive_poses.resize(1);
    collision_objects[1].primitive_poses[0].position.x = 0;
    collision_objects[1].primitive_poses[0].position.y = 0.5;
    collision_objects[1].primitive_poses[0].position.z = 0.05-0.2;
    // END_SUB_TUTORIAL

    collision_objects[1].operation = collision_objects[1].ADD;

    // BEGIN_SUB_TUTORIAL object
    // Define the object that we will be manipulating
    collision_objects[7].header.frame_id = "panda_link0";
    collision_objects[7].id = "object1";

    /* Define the primitive and its dimensions. */
    collision_objects[7].primitives.resize(1);
    collision_objects[7].primitives[0].type = collision_objects[7].primitives[0].BOX;
    collision_objects[7].primitives[0].dimensions.resize(3);
    collision_objects[7].primitives[0].dimensions[0] = 0.02;
    collision_objects[7].primitives[0].dimensions[1] = 0.02;
    collision_objects[7].primitives[0].dimensions[2] = 0.2;

    /* Define the pose of the object. */
    collision_objects[7].primitive_poses.resize(1);
    collision_objects[7].primitive_poses[0].position.x = 0.5;
    collision_objects[7].primitive_poses[0].position.y = 0;
    collision_objects[7].primitive_poses[0].position.z = 0.5;
    // END_SUB_TUTORIAL

    collision_objects[7].operation = collision_objects[7].ADD;

    // BEGIN_SUB_TUTORIAL table2
    // Add the second table where we will be placing the cube.
    /* collision_objects[7].id = "table3";
    collision_objects[7].header.frame_id = "panda_link0"; */

    /* Define the primitive and its dimensions. */
    /* collision_objects[7].primitives.resize(1);
    collision_objects[7].primitives[0].type = collision_objects[7].primitives[0].BOX;
    collision_objects[7].primitives[0].dimensions.resize(3);
    collision_objects[7].primitives[0].dimensions[0] = 0.35;
    collision_objects[7].primitives[0].dimensions[1] = 0.55;
    collision_objects[7].primitives[0].dimensions[2] = 0.38; */

    /* Define the pose of the table. */
    /* collision_objects[7].primitive_poses.resize(1);
    collision_objects[7].primitive_poses[0].position.x = 0;
    collision_objects[7].primitive_poses[0].position.y = 0.5;
    collision_objects[7].primitive_poses[0].position.z = 0.29-0.2; */
    // END_SUB_TUTORIAL

    //collision_objects[7].operation = collision_objects[7].ADD;

    // BEGIN_SUB_TUTORIAL object
    // Define the object that we will be manipulating
    /* collision_objects[2].header.frame_id = "panda_link0";
    collision_objects[2].id = "object"; */

    /* Define the primitive and its dimensions. */
    /* collision_objects[2].primitives.resize(1);
    collision_objects[2].primitives[0].type = collision_objects[1].primitives[0].BOX;
    collision_objects[2].primitives[0].dimensions.resize(3);
    collision_objects[2].primitives[0].dimensions[0] = 0.02;
    collision_objects[2].primitives[0].dimensions[1] = 0.02;
    collision_objects[2].primitives[0].dimensions[2] = 0.2; */

    /* Define the pose of the object. */
    /* collision_objects[2].primitive_poses.resize(1);
    collision_objects[2].primitive_poses[0].position.x = 0.5;
    collision_objects[2].primitive_poses[0].position.y = 0;
    collision_objects[2].primitive_poses[0].position.z = 0.5;
    // END_SUB_TUTORIAL

    collision_objects[2].operation = collision_objects[2].ADD; */
    
    // BEGIN_SUB_TUTORIAL object
    // Define the object that we will be manipulating
    /* collision_objects[3].header.frame_id = "panda_link0";
    collision_objects[3].id = "object1"; */

    /* Define the primitive and its dimensions. */
    /* collision_objects[3].primitives.resize(1);
    collision_objects[3].primitives[0].type = collision_objects[1].primitives[0].BOX;
    collision_objects[3].primitives[0].dimensions.resize(3);
    collision_objects[3].primitives[0].dimensions[0] = 0.02;
    collision_objects[3].primitives[0].dimensions[1] = 0.02;
    collision_objects[3].primitives[0].dimensions[2] = 0.2; */

    /* Define the pose of the object. */
    /* collision_objects[3].primitive_poses.resize(1);
    collision_objects[3].primitive_poses[0].position.x = 0.5;
    collision_objects[3].primitive_poses[0].position.y = 0.2;
    collision_objects[3].primitive_poses[0].position.z = 0.5;
    // END_SUB_TUTORIAL

    collision_objects[3].operation = collision_objects[3].ADD;*/

    planning_scene_interface.applyCollisionObjects(collision_objects);

  }
};
int main(int argc, char** argv)
{
  ros::init(argc, argv, "panda_arm_pick_place");
  ros::NodeHandle nh;
  PickPlace nc = PickPlace(&nh);
  return 0;
}