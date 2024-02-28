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

## Must be called before catkin_package
add_service_files(
        DIRECTORY srv
        FILES
        getIk.srv
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
)

##########
## YAML ##
##########

include(FetchContent)
FetchContent_Declare(
        yaml-cpp
        GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git 
        GIT_TAG yaml-cpp-0.7.0
)
FetchContent_GetProperties(yaml-cpp)

if(NOT yaml-cpp_POPULATED)
  message(STATUS "Fetching yaml-cpp...")
  FetchContent_Populate(yaml-cpp)
  add_subdirectory(${yaml-cpp_SOURCE_DIR} ${yaml-cpp_BINARY_DIR})
endif()

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

## TODO: Check diff between executable and lib 
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
    yaml-cpp::yaml-cpp 
)
