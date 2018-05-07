#!/usr/bin/env python

# robot_commander.py
# Copyright (C) 2017 Niryo
# All rights reserved.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

import rospy, sys
import moveit_commander
import actionlib
import threading
# Lib
from niryo_one_commander.command_type import CommandType
from niryo_one_commander.command_status import CommandStatus
from niryo_one_commander.robot_commander_exception import RobotCommanderException
from niryo_one_commander.position.position import Position 
from niryo_one_commander.position.position_command_type import PositionCommandType
from position_manager import PositionManager
# Messages
from std_msgs.msg import Empty
from niryo_one_msgs.msg import RobotMoveCommand
from std_msgs.msg import Bool

# Services
from std_srvs.srv import SetBool
from niryo_one_msgs.srv import RobotMove
from niryo_one_msgs.srv import ManagePosition 
from niryo_one_msgs.srv import GetInt
# Action msgs
from niryo_one_msgs.msg import RobotMoveAction
from niryo_one_msgs.msg import RobotMoveGoal
from niryo_one_msgs.msg import RobotMoveResult
# Commanders
from arm_commander import ArmCommander
from tool_commander import ToolCommander
from niryo_one_commander.move_group_arm import MoveGroupArm
from niryo_one_commander.validate_params import ValidateParameters

# State publisher
from niryo_one_robot_state_publisher import NiryoRobotStatePublisher


"""
This class handles the arm and tools through a service interface 
- before you execute a command here, you need to validate params
  and check if no other action is being processed

"""

class RobotCommander:

    def compute_and_execute_plan(self):
        plan = self.move_group_arm.compute_plan()
        if not plan : 
            raise RobotCommanderException(
                CommandStatus.PLAN_FAILED, "Moveit failed to compute the plan.")

	self.reset_controller()
        rospy.loginfo("Send Moveit trajectory")
        return self.arm_commander.execute_plan(plan)

    def set_plan_and_execute(self, traj):
        self.reset_controller()
        rospy.loginfo("Send newly set trajectory to execute")
        if traj == None :
            raise RobotCommanderException(
                CommandStatus.PLAN_FAILED, "Moveit failed to execute plan.")  
        return self.arm_commander.execute_plan(traj.trajectory)

    def reset_controller(self):
        msg = Empty() 
        self.reset_controller_pub.publish(msg)

    def __init__(self, position_manager, trajectory_manager):
        self.trajectory_manager = trajectory_manager
        self.pos_manager=position_manager
        moveit_commander.roscpp_initialize(sys.argv)

        # Load all the sub-commanders
        self.move_group_arm = MoveGroupArm()
        self.arm_commander = ArmCommander(self.move_group_arm)
        self.tool_commander = ToolCommander()

        self.stop_trajectory_server = rospy.Service(
                'niryo_one/commander/stop_command', SetBool, self.callback_stop_command)

        self.reset_controller_pub = rospy.Publisher('/niryo_one/steppers_reset_controller',
             Empty, queue_size=1)
        
        # robot action server 
        self.server = actionlib.ActionServer('niryo_one/commander/robot_action',
            RobotMoveAction, self.on_goal, self.on_cancel, auto_start=False)     
        self.current_goal_handle = None
        self.learning_mode_on = False 
        self.joystick_enabled = False
        self.is_active_server = rospy.Service(
                'niryo_one/commander/is_active', GetInt, self.callback_is_active)

        self.learning_mode_subscriber = rospy.Subscriber(
                '/niryo_one/learning_mode', Bool, self.callback_learning_mode)
        self.joystick_enabled_subscriber = rospy.Subscriber('/niryo_one/joystick_interface/is_enabled', 
                Bool, self.callback_joystick_enabled)
        
        self.validation = rospy.get_param("/niryo_one/robot_command_validation")
        self.validate_parameters = ValidateParameters(self.validation, self.pos_manager, self.trajectory_manager)

    def set_saved_position(self, cmd):
        rospy.loginfo("set saved position")
        pos = self.pos_manager.get_position(cmd.saved_position_name)
        self.arm_commander.set_joint_target(pos.joints) 

    def set_saved_trajectory(self, cmd):
        traj = self.trajectory_manager.get_trajectory(cmd.saved_trajectory_id) 
        status,message = self.set_plan_and_execute(traj.trajectory_plan)
        return status,message 
    
    def execute_command(self, cmd):
        cmd_type = cmd.cmd_type
        status = CommandStatus.ROS_ERROR
        message = "Unknown problem occured during command execution"

        if cmd_type == CommandType.TOOL:
            self.tool_commander.send_tool_command(cmd.tool_cmd)
            status = CommandStatus.SUCCESS # todo get status from send_tool_command
            message = "Tool command has been successfully processed"
        else: # move command
            if cmd_type == CommandType.EXECUTE_TRAJ:
                status, message = self.set_plan_and_execute(cmd.Trajectory)
            elif cmd_type == CommandType.SAVED_TRAJECTORY: 
                status,message = self.set_saved_trajectory(cmd)
                rospy.loginfo('command execute traj saved set saved trajectory ')

            else:
                if cmd_type == CommandType.JOINTS:
                    self.arm_commander.set_joint_target(cmd.joints)
                elif cmd_type == CommandType.POSE:
                    self.arm_commander.set_pose_target(cmd.position.x, cmd.position.y, cmd.position.z,
                                                       cmd.rpy.roll, cmd.rpy.pitch, cmd.rpy.yaw)
                elif cmd_type == CommandType.POSITION:
                    self.arm_commander.set_position_target(cmd.position.x, cmd.position.y, cmd.position.z)
                elif cmd_type == CommandType.RPY:
                    self.arm_commander.set_rpy_target(cmd.rpy.roll, cmd.rpy.pitch, cmd.rpy.yaw)
                elif cmd_type == CommandType.SHIFT_POSE:
                    self.arm_commander.set_shift_pose_target(cmd.shift.axis_number, cmd.shift.value)
                elif cmd_type == CommandType.POSE_QUAT:
                    self.arm_commander.set_pose_quat_target(cmd.pose_quat)
                elif cmd_type == CommandType.SAVED_POSITION: 
                    self.set_saved_position(cmd)

            

                status, message = self.compute_and_execute_plan()
        return (status, message)

    def cancel_command(self):
        self.arm_commander.stop_current_plan()
        self.tool_commander.stop_tool_command() # todo handle goal cancelation for tools (client side)

    def callback_stop_command(self, req):
        self.cancel_command()
        return True, "Command stopped"

 # robot action server functions 
    # Check if no other command is being processed
    # - Validate params
    # - Execute command and return status + message
    # - Cancel command if asked
    def start(self):
        self.server.start()
        rospy.loginfo("Action Server started (Robot Commander)")
    
    def create_result(self, status, message):
        result = RobotMoveResult()
        result.status = status
        result.message = message
        return result
   
    def callback_learning_mode(self, msg):
        activate = msg.data

        if not self.learning_mode_on and activate:
            self.cancel_current_command()

        self.learning_mode_on = activate

    def callback_joystick_enabled(self, msg):
        self.joystick_enabled = msg.data

    def callback_is_active(self, req):
        if self.current_goal_handle is not None:
            return 1
        return 0
     
    def on_goal(self, goal_handle):
        rospy.loginfo("Robot Action Server - Received goal. Check if exists")

        # Check if learning mode ON
        if self.learning_mode_on:
            result = self.create_result(CommandStatus.LEARNING_MODE_ON,
                 "You need to deactivate learning mode to execute a new command")
            goal_handle.set_rejected(result)
            return

        # Check if joystick enabled
        if self.joystick_enabled:
            result = self.create_result(CommandStatus.JOYSTICK_ENABLED, 
                "You need to deactivate joystick to execute a new command")
            goal_handle.set_rejected(result)
            return

        # check if still have a goal -> set_rejected() 
        if self.current_goal_handle is not None:
            result = self.create_result(CommandStatus.GOAL_STILL_ACTIVE, 
                "Current command is still active. Cancel it if you want to execute a new one")
            goal_handle.set_rejected(result)
            return
      
        # validate parameters -> set_rejected (msg : validation or commander error)
        try:
            rospy.loginfo("Robot Acton Sever - checking paramter Validity")
            self.validate_parameters.validate_params(goal_handle.goal.goal.cmd)
        except RobotCommanderException as e:
            result = self.create_result(e.status, e.message)
            goal_handle.set_rejected(result)
            rospy.loginfo("Robot Action Server - Invalid parameters")
            return
        
        # set accepted
        self.current_goal_handle = goal_handle
        self.current_goal_handle.set_accepted()
        rospy.loginfo("Robot Action Server - Goal has been accepted")

        # Launch compute + execution in a new thread
        w = threading.Thread(name="worker", target=self.execute_command_action)
        w.start()
        rospy.loginfo("Robot Action Server : Executing command in a new thread")

    def on_cancel(self, goal_handle):
        rospy.loginfo("Received cancel command")

        if goal_handle == self.current_goal_handle:
            self.cancel_current_command()
        else:
            rospy.loginfo("Robot Action Server - No current goal, nothing to do")

    def execute_command_action(self):
        cmd = self.current_goal_handle.goal.goal.cmd 
        rospy.loginfo("passing to executing command")
        result = self.create_result(CommandStatus.ROS_ERROR, "error with executing commad")
        response = None
        try: 
            (status, message) = self.execute_command(cmd)
            response = self.create_result(status, message)
            result = response 
        except RobotCommanderException :
            rospy.loginfo ("error with executing command ") 

        if not response:
            self.current_goal_handle.set_aborted(result)
            rospy.loginfo("Execution has been aborted")
        elif response.status == CommandStatus.SUCCESS:
            self.current_goal_handle.set_succeeded(result)
            rospy.loginfo("Goal has been set as succeeded")
        elif response.status == CommandStatus.STOPPED:
            self.current_goal_handle.set_canceled(result)
            rospy.loginfo("Goal has been successfully canceled")
        else:
            self.current_goal_handle.set_aborted(result)
            rospy.loginfo("Unknown result, goal has been set as aborted")
        self.current_goal_handle = None

    
    # Send a cancel signal to Moveit interface
    def cancel_current_command(self):
        try:
            self.cancel_command ()
        except RobotCommanderException:
            rospy.logwarn("Could not cancel current command ")
    



if __name__ == '__main__':
    
    pass




