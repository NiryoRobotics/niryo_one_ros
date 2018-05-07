#!/usr/bin/env python

# validate_params.py
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



import rospy 
from math import sqrt

from niryo_one_commander.robot_commander_exception import RobotCommanderException
from niryo_one_commander.command_status import CommandStatus
from niryo_one_commander.command_type import CommandType

class ValidateParameters(): 

    def __init__(self, validation, position_manager, trajectory_manager):
        self.validation = validation 
        self.pos_manager = position_manager
        self.trajectory_manager = trajectory_manager
    
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
        elif CommandType.SAVED_TRAJECTORY:
            self.validate_saved_trajectory(cmd)

        else:
            raise RobotCommanderException(CommandStatus.INVALID_PARAMETERS, "Wrong command type")


    def validate_saved_trajectory(self, cmd): 
        rospy.loginfo("Checking saved trajectory validity")
        saved_traj = self.trajectory_manager.get_trajectory(cmd.saved_trajectory_id)
        if saved_traj == None :
            raise RobotCommanderException(CommandStatus.INVALID_PARAMETERS, "Saved trajectory  not found") 
        self.validate_trajectory(saved_traj.trajectory_plan)
         

    def validate_saved_position(self, position_name):
        rospy.loginfo("Checking joints validity")
        saved_position = self.pos_manager.get_position(position_name)
        if saved_position == None :
            raise RobotCommanderException(CommandStatus.INVALID_PARAMETERS, "Saved position not found") 
        self.validate_joints(saved_position.joints)
          
    def validate_trajectory(self, plan):
        rospy.loginfo("Checking trajectory validity")
        #Do soemthing here to check if the trajectory is valid
        n =len(plan.trajectory.joint_trajectory.points)
        for i in range(0,n-1): 
            self.validate_joints(plan.trajectory.joint_trajectory.points[i].positions)
        
       
    
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
