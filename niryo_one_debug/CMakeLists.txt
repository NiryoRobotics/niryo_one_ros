cmake_minimum_required(VERSION 2.8.3)
project(niryo_one_debug)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  dynamixel_sdk
  mcp_can_rpi
  niryo_one_msgs
  roscpp
  rospy
  std_msgs
)

find_package(Boost REQUIRED COMPONENTS system)


catkin_package(
  CATKIN_DEPENDS 
    dynamixel_sdk 
    mcp_can_rpi 
    niryo_one_msgs 
    roscpp 
    rospy 
    std_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
