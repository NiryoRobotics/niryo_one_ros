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

# Lib
from niryo_one_commander.command_type import CommandType
from niryo_one_commander.command_status import CommandStatus
from niryo_one_commander.robot_commander_exception import RobotCommanderException

# Messages
from std_msgs.msg import Empty
from niryo_one_msgs.msg import RobotMoveCommand

# Services
from std_srvs.srv import SetBool
from niryo_one_msgs.srv import RobotMove

# Commanders
from arm_moveit_commander import ArmMoveitCommander
from tool_commander import ToolCommander

# ROS action interface
from robot_action_server import RobotActionServer

# State publisher
from niryo_one_robot_state_publisher import NiryoRobotStatePublisher

"""
This class handles the arm and tools through a service interface 
- before you execute a command here, you need to validate params
  and check if no other action is being processed

"""

class RobotCommander:

    def compute_and_execute_plan(self):
        self.arm_commander.compute_plan()
        self.reset_controller()
        rospy.loginfo("Send Moveit trajectory")
        return self.arm_commander.execute_plan()
    
    def set_plan_and_execute(self, plan):
        self.arm_commander.set_next_plan(plan)
        self.reset_controller()
        rospy.loginfo("Send newly set trajectory to execute")
        return self.arm_commander.execute_plan()

    def reset_controller(self):
        msg = Empty() 
        self.reset_controller_pub.publish(msg)

    def create_response(self, status, message):
        return {'status': status, 'message': message}
    
    def __init__(self):

        moveit_commander.roscpp_initialize(sys.argv)

        # Load all the sub-commanders
        self.arm_commander = ArmMoveitCommander()
        self.tool_commander = ToolCommander()

        # Publish robot state (position, orientation, gripper)
        self.niryo_one_robot_state_publisher = NiryoRobotStatePublisher()

        self.stop_trajectory_server = rospy.Service(
                'niryo_one/commander/stop_command', SetBool, self.callback_stop_command)

        self.robot_command_server = rospy.Service('niryo_one/commander/execute_command',
                RobotMove, self.callback_robot_command)

        self.reset_controller_pub = rospy.Publisher('/niryo_one/steppers_reset_controller', Empty, queue_size=1)
        
        # Load ROS action interface
        robot_action_server = RobotActionServer()
        robot_action_server.start()

    def callback_robot_command(self, req):
        cmd = req.cmd
        try:
            return self.execute_command(cmd)
        except RobotCommanderException as e:
            return self.create_response(e.status, e.message)
        
    def callback_set_plan_and_execute(self, req):
        plan = req.Trajectory
        try: 
            return self.set_plan_and_execute(plan)
        except RobotCommanderException as e:
            return self.create_response(e.status, e.message)
        

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
                status, message = self.set_plan_and_execute(cmd.plan)
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
            
                status, message = self.compute_and_execute_plan()

        result = self.create_response(status, message)
        return result

    def cancel_command(self):
        self.arm_commander.stop_current_plan()
        self.tool_commander.stop_tool_command() # todo handle goal cancelation for tools (client side)

    def callback_stop_command(self, req):
        self.cancel_command()
        return True, "Command stopped"


if __name__ == '__main__':
    rospy.init_node('robot_commander')
    rc = RobotCommander()
    rospy.spin()






