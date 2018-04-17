#!/usr/bin/env python

# robot_action_server.py
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
import actionlib
import threading
from math import sqrt

# Lib
from niryo_one_commander.command_type import CommandType
from niryo_one_commander.command_status import CommandStatus
from niryo_one_commander.robot_commander_exception import RobotCommanderException
from position_manager import PositionManager

# Messages
from std_msgs.msg import Bool
from niryo_one_msgs.msg import RobotMoveCommand

# Services
from std_srvs.srv import SetBool
from niryo_one_msgs.srv import GetInt
from niryo_one_msgs.srv import RobotMove

# Action msgs
from niryo_one_msgs.msg import RobotMoveAction
from niryo_one_msgs.msg import RobotMoveGoal
from niryo_one_msgs.msg import RobotMoveResult
#
# This class is the main interface to use niryo_one_commander
# It implements an action server that will :
# - Check if no other command is being processed
# - Validate params
# - Execute command and return status + message
# - Cancel command if asked
#

class RobotActionServer:

    def __init__(self, position_manager):
        
        self.pos_manager = position_manager
        self.server = actionlib.ActionServer('niryo_one/commander/robot_action', RobotMoveAction, self.on_goal, self.on_cancel, auto_start=False)
    
        self.current_goal_handle = None

        self.learning_mode_on = False 
        self.joystick_enabled = False

        # Load validation params from rosparams
        self.validation = rospy.get_param("/niryo_one/robot_command_validation")
        
        rospy.loginfo("Waiting for service 'niryo_one/commander/execute_command'...")
        rospy.wait_for_service('niryo_one/commander/execute_command')
        rospy.loginfo("Service found : 'niryo_one/commander/execute_command'")
        
        self.is_active_server = rospy.Service(
                'niryo_one/commander/is_active', GetInt, self.callback_is_active)

        self.learning_mode_subscriber = rospy.Subscriber(
                '/niryo_one/learning_mode', Bool, self.callback_learning_mode)
        self.joystick_enabled_subscriber = rospy.Subscriber('/niryo_one/joystick_interface/is_enabled', 
                Bool, self.callback_joystick_enabled)

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

    def start(self):
        self.server.start()
        rospy.loginfo("Action Server started (Robot Commander)")

    def create_result(self, status, message):
        result = RobotMoveResult()
        result.status = status
        result.message = message
        return result

    def on_goal(self, goal_handle):
        rospy.loginfo("Robot Action Server - Received goal. Check if exists")

        # Check if learning mode ON
        if self.learning_mode_on:
            result = self.create_result(CommandStatus.LEARNING_MODE_ON, "You need to deactivate learning mode to execute a new command")
            goal_handle.set_rejected(result)
            return

        # Check if joystick enabled
        if self.joystick_enabled:
            result = self.create_result(CommandStatus.JOYSTICK_ENABLED, "You need to deactivate joystick to execute a new command")
            goal_handle.set_rejected(result)
            return

        # check if still have a goal -> set_rejected() 
        if self.current_goal_handle is not None:
            result = self.create_result(CommandStatus.GOAL_STILL_ACTIVE, "Current command is still active. Cancel it if you want to execute a new one")
            goal_handle.set_rejected(result)
            return
      
        # validate parameters -> set_rejected (msg : validation or commander error)
        try:
            rospy.loginfo("Robot Acton Sever - checking paramter Validity")
            self.validate_params(goal_handle.goal.goal.cmd)
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
        w = threading.Thread(name="worker", target=self.execute_command)
        w.start()
        rospy.loginfo("Robot Action Server : Executing command in a new thread")

    def on_cancel(self, goal_handle):
        rospy.loginfo("Received cancel command")

        if goal_handle == self.current_goal_handle:
            self.cancel_current_command()
        else:
            rospy.loginfo("Robot Action Server - No current goal, nothing to do")

    def execute_command(self):
        cmd = self.current_goal_handle.goal.goal.cmd
        
        rospy.wait_for_service('niryo_one/commander/execute_command')
        result = self.create_result(CommandStatus.ROS_ERROR, "Ros error with service")
        response = None
        try:
            exec_cmd = rospy.ServiceProxy('niryo_one/commander/execute_command', RobotMove)
            response = exec_cmd(cmd)
            result = self.create_result(response.status, response.message)
        except rospy.ServiceException as e:
            rospy.loginfo(e)

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
        rospy.wait_for_service('niryo_one/commander/stop_command')
        try:
            cancel_command = rospy.ServiceProxy('niryo_one/commander/stop_command', SetBool)
            cancel_command(True)
        except rospy.ServiceException as e:
            rospy.logwarn("Could not cancel current command : " + str(e))
    
    def validate_params(self, cmd):
        cmd_type = cmd.cmd_type
        if cmd_type == CommandType.JOINTS:
            self.validate_joints(cmd.joints)
        elif cmd_type == CommandType.POSE:
            self.validate_position(cmd.position)
            self.validate_orientation(cmd.rpy)
        elif cmd_type == CommandType.POSITION:
            self.validate_position(cmd.position)
        elif cmd_type == CommandType.RPY:
            self.validate_orientation(cmd.rpy)
        elif cmd_type == CommandType.SHIFT_POSE:
            self.validate_shift_pose(cmd.shift)
        elif cmd_type == CommandType.EXECUTE_TRAJ:
            self.validate_trajectory(cmd.Trajectory)
        elif cmd_type == CommandType.TOOL:
            self.validate_tool_command(cmd.tool_cmd)
        elif cmd_type == CommandType.POSE_QUAT:
            self.validate_position(cmd.pose_quat.position)
            self.validate_orientation_quaternion(cmd.pose_quat.orientation)
        elif cmd_type == CommandType.SAVED_POSITION: 
            self.validate_saved_position(cmd.saved_position_name)
        else:
            raise RobotCommanderException(CommandStatus.INVALID_PARAMETERS, "Wrong command type")

    def validate_saved_position(self, position_name):
        rospy.loginfo("Checking joints validity")
        saved_position = self.pos_manager.get_position(position_name)
        if saved_position == None :
            raise RobotCommanderException(CommandStatus.INVALID_PARAMETERS, "command is empty") 
        self.validate_joints(saved_position.joints)
          
    def validate_trajectory(self, plan):
        rospy.loginfo("Checking trajectory validity")
        #Do soemthing here to check if the trajectory is valid
        dummy = 0
    
    def validate_joints(self, joint_array):
        v = self.validation['joint_limits']

        if len(joint_array) != 6:
            raise RobotCommanderException(CommandStatus.INVALID_PARAMETERS, "Joint array must have 6 joints")
        if( joint_array[0] < v['j1']['min'] or joint_array[0] > v['j1']['max']):
            raise RobotCommanderException(CommandStatus.INVALID_PARAMETERS, 
                    "joint 1 not in range ( " + str(v['j1']['min']) + " , " + str(v['j1']['max']) + " )")
        if( joint_array[1] < v['j2']['min'] or joint_array[1] > v['j2']['max']):
            raise RobotCommanderException(CommandStatus.INVALID_PARAMETERS,
                    "joint 2 not in range ( " + str(v['j2']['min']) + " , " + str(v['j2']['max']) + " )")
        if( joint_array[2] < v['j3']['min'] or joint_array[2] > v['j3']['max']):
            raise RobotCommanderException(CommandStatus.INVALID_PARAMETERS,
                    "joint 3 not in range ( " + str(v['j3']['min']) + " , " + str(v['j3']['max']) + " )")
        if( joint_array[3] < v['j4']['min'] or joint_array[3] > v['j4']['max']):
            raise RobotCommanderException(CommandStatus.INVALID_PARAMETERS, 
                    "joint 4 not in range ( " + str(v['j4']['min']) + " , " + str(v['j4']['max']) + " )")
        if( joint_array[4] < v['j5']['min'] or joint_array[4] > v['j5']['max']):
            raise RobotCommanderException(CommandStatus.INVALID_PARAMETERS,
                    "joint 5 not in range ( " + str(v['j5']['min']) + " , " + str(v['j5']['max']) + " )")
        if( joint_array[5] < v['j6']['min'] or joint_array[5] > v['j6']['max']):
            raise RobotCommanderException(CommandStatus.INVALID_PARAMETERS,
                    "joint 6 not in range ( " + str(v['j6']['min']) + " , " + str(v['j6']['max']) + " )")

    def validate_position(self, position):
        v = self.validation['position_limits']

        if (position.x < v['x']['min'] or position.x > v['x']['max']):
            raise RobotCommanderException(CommandStatus.INVALID_PARAMETERS,
                    "x not in range ( " + str(v['x']['min']) + " , " + str(v['x']['max']) + " )")
        if (position.y < v['y']['min'] or position.y > v['y']['max']):
            raise RobotCommanderException(CommandStatus.INVALID_PARAMETERS,
                    "y not in range ( " + str(v['y']['min']) + " , " + str(v['y']['max']) + " )")
        if (position.z < v['z']['min'] or position.z > v['z']['max']):
            raise RobotCommanderException(CommandStatus.INVALID_PARAMETERS,
                    "z not in range ( " + str(v['z']['min']) + " , " + str(v['z']['max']) + " )")

    def validate_orientation(self, orientation):
        v = self.validation['rpy_limits']

        if (orientation.roll < v['roll']['min'] or orientation.roll > v['roll']['max']):
            raise RobotCommanderException(CommandStatus.INVALID_PARAMETERS, 
                    "rot. x (roll) not in range ( " + str(v['roll']['min']) + " , " + str(v['roll']['max']) + " )")
        if (orientation.pitch < v['pitch']['min'] or orientation.pitch > v['pitch']['max']):
            raise RobotCommanderException(CommandStatus.INVALID_PARAMETERS,
                    "rot.y (pitch) not in range ( " + str(v['pitch']['min']) + " , " + str(v['pitch']['max']) + " )")
        if (orientation.yaw < v['yaw']['min'] or orientation.yaw > v['yaw']['max']):
            raise RobotCommanderException(CommandStatus.INVALID_PARAMETERS,
                    "rot.z (yaw) not in range ( " + str(v['yaw']['min']) + " , " + str(v['yaw']['max']) + " )")
            
    def validate_orientation_quaternion(self, quat):
        norm = quat.x*quat.x +quat.y*quat.y + quat.z*quat.z + quat.w*quat.w
        norm = sqrt(norm)
        if (abs(norm-1.0) > 0.001):
            raise RobotCommanderException(CommandStatus.INVALID_PARAMETERS,
                    "Quaternian is not normalised.")

    def validate_shift_pose(self, shift):
        if (shift.axis_number not in [0,1,2,3,4,5]):
            raise RobotCommanderException(CommandStatus.INVALID_PARAMETERS, "shift axis number not in [0,1,2,3,4,5]")
        if (shift.value == 0 or shift.value < -1.0 or shift.value > 1.0):
            raise RobotCommanderException(CommandStatus.INVALID_PARAMETERS, "shift value can't be null and < -1 or > 1")

    def validate_tool_command(self, cmd):
        # for now, validation in ToolsController
        pass

