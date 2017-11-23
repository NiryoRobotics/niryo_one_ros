cmake_minimum_required(VERSION 2.8.3)
project(niryo_one_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  actionlib_msgs
  message_generation
)

add_message_files(
  FILES
  RPY.msg
  ShiftPose.msg
  RobotState.msg
  RobotMoveCommand.msg
  ToolCommand.msg
  BlocklyCommand.msg
  ProcessState.msg
  HardwareStatus.msg
  LogStatus.msg
  DigitalIOState.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  SetInt.srv
  RobotMove.srv
  ManageProcess.srv
  SetLeds.srv
  GetInt.srv
  PingDxlTool.srv
  OpenGripper.srv
  CloseGripper.srv
  PullAirVacuumPump.srv
  PushAirVacuumPump.srv
  SetDigitalIO.srv
)

add_action_files(
  DIRECTORY action
  FILES 
  JoystickJoints.action
  RobotMove.action
  Tool.action
  Blockly.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime actionlib_msgs geometry_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

