cmake_minimum_required(VERSION 2.8.3)
project(niryo_one_tools)

find_package(catkin REQUIRED)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(
	PROGRAMS
	  scripts/tool_controller.py
	  scripts/tool_ros_command_interface.py
	  scripts/tools.py
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
