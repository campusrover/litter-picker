cmake_minimum_required(VERSION 3.0.2)
project(litter_picker)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  actionlib_msgs
  turtlebot3_navigation
  turtlebot3_slam
  message_generation
)

add_action_files(
    FILES
    Navigation.action
    Rotation.action
)

generate_messages(
    DEPENDENCIES
    actionlib_msgs
    std_msgs
)

catkin_package(
    CATKIN_DEPENDS
    actionlib_msgs
    std_msgs
    message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
