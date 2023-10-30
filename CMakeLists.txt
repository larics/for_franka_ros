cmake_minimum_required(VERSION 3.0.2)
project(for_franka_ros)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    franka_gazebo 
    franka_control 
    franka_description
    franka_example_controllers
    franka_msgs
    panda_moveit_config
    geometry_msgs
    moveit_core
    moveit_visual_tools
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_ros_move_group
    moveit_msgs
    moveit_servo
    roscpp
    std_msgs
    tf
    tf_conversions
    trajectory_msgs
)

find_package(Eigen3 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES
    ${PROJECT_NAME}_lib
  CATKIN_DEPENDS 
    moveit_core
    moveit_visual_tools
    moveit_ros_planning
    moveit_ros_planning_interface
    moveit_msgs
  DEPENDS
    Eigen3
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR}
)

## Declare a C++ library
set(PROJECT_LIB_FILES 
    src/control_arm.cpp
)

add_library(
        ${PROJECT_NAME}_lib
        ${PROJECT_LIB_FILES}
)

add_executable(
        ${PROJECT_NAME}_node
        src/control_arm_node.cpp
)

target_link_libraries(
    ${PROJECT_NAME}_node
    ${PROJECT_NAME}_lib
    ${EIGEN3_LIBRARIES}
    ${catkin_LIBRARIES}
)
