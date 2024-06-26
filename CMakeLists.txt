## Nice GitLab tutorial about CMake: https://cliutils.gitlab.io/modern-cmake/chapters/basics.html
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
find_package(yaml-cpp REQUIRED)

## Must be called before catkin_package
add_service_files(
        DIRECTORY srv
        FILES
        getIk.srv
        changeState.srv
)

generate_messages(
        DEPENDENCIES std_msgs geometry_msgs sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include srv
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
    yaml-cpp
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR}
)

## TODO: Check diff between executable and lib 
add_library(
        control_arm_lib
        src/control_arm.cpp
)

add_executable(
        ${PROJECT_NAME}_node
        src/control_arm_node.cpp
)

target_link_libraries(
    ${PROJECT_NAME}_node
    control_arm_lib
    ${EIGEN3_LIBRARIES}
    ${catkin_LIBRARIES}
    yaml-cpp
)

add_library(
        control_arm_servo_lib
        src/control_arm_servo.cpp
        )

add_executable(
        control_arm_servo_node
        src/control_arm_servo_node.cpp
)

target_link_libraries(
    control_arm_servo_node
    control_arm_servo_lib
    ${EIGEN3_LIBRARIES}
    ${catkin_LIBRARIES}
    yaml-cpp
)


