cmake_minimum_required(VERSION 2.8.3)
project(niryo_one_gazebo)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
)
catkin_package()
include_directories(
 ${catkin_INCLUDE_DIRS}
)
