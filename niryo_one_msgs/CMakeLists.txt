cmake_minimum_required(VERSION 2.8.3)
project(niryo_one_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  actionlib_msgs
  message_generation
  moveit_msgs
)

add_message_files(
  FILES
  RPY.msg
  ShiftPose.msg
  RobotState.msg
  RobotMoveCommand.msg
  ToolCommand.msg
  ProcessState.msg
  HardwareStatus.msg
  LogStatus.msg
  DigitalIOState.msg
  SoftwareVersion.msg
  Sequence.msg
  SequenceAutorunStatus.msg
  TrajectoryPlan.msg
  MatlabMoveResult.msg
  Position.msg
  Trajectory.msg
  ObjectPose.msg
  ConveyorFeedback.msg
)

add_service_files(
  DIRECTORY srv
  FILES
  ChangeHardwareVersion.srv
  ChangeMotorConfig.srv
  ControlConveyor.srv
  CloseGripper.srv
  DebugColorDetection.srv
  DebugMarkers.srv
  EditGrip.srv
  EditWorkspace.srv
  GetCalibrationCam.srv
  GetDigitalIO.srv
  GetInt.srv
  GetPositionList.srv
  GetSequenceList.srv
  GetTargetPose.srv
  GetTrajectoryList.srv
  GetWorkspaceList.srv
  GetWorkspaceRatio.srv
  GetWorkspaceRobotPoses.srv
  ObjDetection.srv
  ManagePosition.srv
  ManageProcess.srv
  ManageSequence.srv
  ManageTrajectory.srv
  OpenGripper.srv
  PingDxlTool.srv
  PullAirVacuumPump.srv
  PushAirVacuumPump.srv
  RobotMove.srv
  SendCustomDxlValue.srv
  SetBool.srv
  SetCalibrationCam.srv
  SetConveyor.srv
  SetDigitalIO.srv
  SetInt.srv
  SetLeds.srv
  SetSequenceAutorun.srv
  SetString.srv
  TakePicture.srv
  UpdateConveyorId.srv
)

add_action_files(
  DIRECTORY action
  FILES
  JoystickJoints.action
  RobotMove.action
  Tool.action
  Sequence.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  actionlib_msgs
  moveit_msgs
  sensor_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp rospy std_msgs message_runtime actionlib_msgs geometry_msgs moveit_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
