#!/usr/bin/env python

# niryo_one_api.py
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
import actionlib

from actionlib_msgs.msg import GoalStatus

from std_msgs.msg import Int32
from std_msgs.msg import String

from niryo_one_msgs.msg import RobotMoveAction
from niryo_one_msgs.msg import RobotMoveGoal
from niryo_one_msgs.msg import HardwareStatus

from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from niryo_one_msgs.msg import HardwareStatus
from niryo_one_msgs.msg import RobotState
from niryo_one_msgs.msg import DigitalIOState

from niryo_one_msgs.srv import SetInt
from niryo_one_msgs.srv import GetDigitalIO
from niryo_one_msgs.srv import SetDigitalIO
from niryo_one_msgs.srv import GetPositionList

from niryo_one_commander.command_type import CommandType as MoveCommandType

# Tools IDs (need to match tools ids in niryo_one_tools package)
TOOL_NONE = 0
TOOL_GRIPPER_1_ID = 11
TOOL_GRIPPER_2_ID = 12
TOOL_GRIPPER_3_ID = 13
TOOL_ELECTROMAGNET_1_ID = 30
TOOL_VACUUM_PUMP_1_ID = 31

# GPIOs
PIN_MODE_OUTPUT = 0
PIN_MODE_INPUT = 1
PIN_HIGH = 1
PIN_LOW = 0

GPIO_1A = 2
GPIO_1B = 3
GPIO_1C = 16
GPIO_2A = 26
GPIO_2B = 19
GPIO_2C = 6

SW_1 = 12
SW_2 = 13

# Status
OK    = 200
ERROR = 400

# for shift_pose function
AXIS_X    = 0
AXIS_Y    = 1
AXIS_Z    = 2
ROT_ROLL  = 3
ROT_PITCH = 4
ROT_YAW   = 5


class NiryoOneException(Exception): pass

class NiryoOne:
    
    # Singleton
    instance = None

    def __init__(self):
        if not NiryoOne.instance:
            NiryoOne.instance = NiryoOne.__NiryoOne()

    def __getattr__(self, name):
        return getattr(self.instance, name)

    class __NiryoOne:

        def __init__(self):
            self.service_timeout = rospy.get_param("/niryo_one/python_api/service_timeout")
            self.action_connection_timeout = rospy.get_param("/niryo_one/python_api/action_connection_timeout")
            self.action_execute_timeout = rospy.get_param("/niryo_one/python_api/action_execute_timeout")
            self.action_preempt_timeout = rospy.get_param("/niryo_one/python_api/action_preempt_timeout")
            self.tool_command_list = rospy.get_param("/niryo_one_tools/command_list") 

            # Subscribers
            self.joint_states_sub = rospy.Subscriber('/joint_states', JointState, self.sub_joint_states)
            self.robot_state_sub  = rospy.Subscriber('/niryo_one/robot_state', RobotState, self.sub_robot_state)
            self.hardware_status_sub  = rospy.Subscriber('/niryo_one/hardware_status', HardwareStatus, self.sub_hardware_status)
            self.learning_mode_sub  = rospy.Subscriber('/niryo_one/learning_mode', Bool, self.sub_learning_mode)
            self.digital_io_state_sub = rospy.Subscriber('/niryo_one/rpi/digital_io_state', DigitalIOState, self.sub_digital_io_state)

            # Highlight publisher (to highlight blocks in Blockly interface)
            self.highlight_block_publisher = rospy.Publisher('/niryo_one/blockly/highlight_block', String, queue_size=10)

            # Break point publisher (for break point blocks in Blockly interface)
            self.break_point_publisher = rospy.Publisher('/niryo_one/blockly/break_point', Int32, queue_size=10)

            self.joints = None
            self.pose = None
            self.hw_status = None
            self.learning_mode_on = None
            self.digital_io_state = None
            
            rospy.sleep(0.1)

        #
        # Subscribers callbacks
        #

        def sub_joint_states(self, joint_states):
            self.joints = list(joint_states.position)

        def sub_hardware_status(self, hw_status):
            self.hw_status = hw_status

        def sub_robot_state(self, pose):
            self.pose = pose
        
        def sub_learning_mode(self, learning_mode):
            self.learning_mode_on = learning_mode.data
        
        def sub_digital_io_state(self, digital_io_state):
            self.digital_io_state = digital_io_state

        #
        # Service client
        #

        def call_service(self, service_name, service_msg_type, args):
            
            # Connect to service
            try:
                rospy.wait_for_service(service_name, self.service_timeout)
            except rospy.ROSException, e:
                raise NiryoOneException(e)
            
            # Call service
            try:
                service = rospy.ServiceProxy(service_name, service_msg_type)
                response = service(*args)
                return response
            except rospy.ServiceException, e:
                raise NiryoOneException(e)
        
        #
        # Action client
        #
           
        def execute_action(self, action_name, action_msg_type, goal):
            
            client = actionlib.SimpleActionClient(action_name, action_msg_type)

            # Connect to server
            if not client.wait_for_server(rospy.Duration(self.action_connection_timeout)):
                raise NiryoOneException('Action Server is not up : ' + str(action_name))
        
            # Send goal and check response
            client.send_goal(goal)
            
            if not client.wait_for_result(timeout=rospy.Duration(self.action_execute_timeout)):
                client.cancel_goal()
                client.stop_tracking_goal()
                raise NiryoOneException('Action Server timeout : ' + str(action_name))

            goal_state = client.get_state()
            response = client.get_result()

            if goal_state != GoalStatus.SUCCEEDED:
                client.stop_tracking_goal()

            if goal_state == GoalStatus.REJECTED:
                raise NiryoOneException('Goal has been rejected : ' + str(response.message))
            elif goal_state == GoalStatus.ABORTED:
                raise NiryoOneException('Goal has been aborted : ' + str(response.message))
            elif goal_state != GoalStatus.SUCCEEDED:
                raise NiryoOneException('Error when processing goal : ' + str(response.message))

            return response.message
       
        #    
        # Interface
        #

        def calibrate_auto(self):
            result = self.call_service('niryo_one/calibrate_motors',
                    SetInt, [1])
            if result.status != 200:
                raise NiryoOneException(result.message)
            # Wait until calibration is finished
            rospy.sleep(1)
            calibration_finished = False
            while not calibration_finished:
                try:
                    hw_status = rospy.wait_for_message('niryo_one/hardware_status', 
                            HardwareStatus, timeout=5)
                    if not hw_status.calibration_in_progress:
                        calibration_finished = True
                except rospy.ROSException as e:
                    raise NiryoOneException(str(e))

        def calibrate_manual(self):
            result = self.call_service('niryo_one/calibrate_motors',
                    SetInt, [2])
            if result.status != 200:
                raise NiryoOneException(result.message)
            # Wait until calibration is finished
            rospy.sleep(1)
            calibration_finished = False
            while not calibration_finished:
                try:
                    hw_status = rospy.wait_for_message('niryo_one/hardware_status', 
                            HardwareStatus, timeout=5)
                    if not hw_status.calibration_in_progress:
                        calibration_finished = True
                except rospy.ROSException as e:
                    raise NiryoOneException(str(e))
       
        def activate_learning_mode(self, activate):
            result = self.call_service('niryo_one/activate_learning_mode', 
                    SetInt, [activate])
            if result.status != 200:
                raise NiryoOneException(result.message)
            rospy.sleep(0.1)
        
        def pin_mode(self, pin, mode):
            result = self.call_service('niryo_one/rpi/set_digital_io_mode', 
                    SetDigitalIO, [pin, mode])
            if result.status != 200:
                raise NiryoOneException(result.message)
        
        def digital_write(self, pin, state):
            result = self.call_service('niryo_one/rpi/set_digital_io_state', 
                    SetDigitalIO, [pin, state])
            if result.status != 200:
                raise NiryoOneException(result.message)
 
        def digital_read(self, pin):
            result = self.call_service('niryo_one/rpi/get_digital_io',
                    GetDigitalIO, [pin])
            if result.status != 200:
                raise NiryoOneException(result.message)
            return result.state

        def change_tool(self, tool_id):
            result = self.call_service('niryo_one/change_tool', 
                        SetInt, [int(tool_id)])
            if result.status != 200:
                raise NiryoOneException(result.message)

        def move_pose(self, x, y, z, roll, pitch, yaw):
            goal = RobotMoveGoal()
            goal.cmd.cmd_type = MoveCommandType.POSE
            goal.cmd.position.x = x
            goal.cmd.position.y = y
            goal.cmd.position.z = z
            goal.cmd.rpy.roll = roll
            goal.cmd.rpy.pitch = pitch
            goal.cmd.rpy.yaw = yaw
            return self.execute_action('niryo_one/commander/robot_action', RobotMoveAction, goal)

        def move_joints(self, joints):
            goal = RobotMoveGoal()
            goal.cmd.cmd_type = MoveCommandType.JOINTS
            goal.cmd.joints = joints
            return self.execute_action('niryo_one/commander/robot_action', RobotMoveAction, goal)

        def shift_pose(self, axis, value):
            goal = RobotMoveGoal()
            goal.cmd.cmd_type = MoveCommandType.SHIFT_POSE
            goal.cmd.shift.axis_number = axis
            goal.cmd.shift.value = value
            return self.execute_action('niryo_one/commander/robot_action', RobotMoveAction, goal)

        def set_arm_max_velocity(self, percentage):
            result = self.call_service('/niryo_one/commander/set_max_velocity_scaling_factor',
                    SetInt, [percentage])
            if result.status != 200:
                raise NiryoOneException(result.message)

        def enable_joystick(self, enable):
            result = self.call_service('/niryo_one/joystick_interface/enable',
                    SetInt, [enable])
            if result.status != 200:
                raise NiryoOneException(result.message)

        def open_gripper(self, gripper_id, speed):
            goal = RobotMoveGoal()
            goal.cmd.cmd_type = MoveCommandType.TOOL
            goal.cmd.tool_cmd.tool_id = int(gripper_id)
            goal.cmd.tool_cmd.cmd_type = self.tool_command_list['open_gripper']
            goal.cmd.tool_cmd.gripper_open_speed = speed
            return self.execute_action('niryo_one/commander/robot_action', RobotMoveAction, goal)

        def close_gripper(self, gripper_id, speed):
            goal = RobotMoveGoal()
            goal.cmd.cmd_type = MoveCommandType.TOOL
            goal.cmd.tool_cmd.tool_id = int(gripper_id)
            goal.cmd.tool_cmd.cmd_type = self.tool_command_list['close_gripper']
            goal.cmd.tool_cmd.gripper_close_speed = speed
            return self.execute_action('niryo_one/commander/robot_action', RobotMoveAction, goal)
        
        def pull_air_vacuum_pump(self, vacuum_pump_id):
            goal = RobotMoveGoal()
            goal.cmd.cmd_type = MoveCommandType.TOOL
            goal.cmd.tool_cmd.tool_id = int(vacuum_pump_id)
            goal.cmd.tool_cmd.cmd_type = self.tool_command_list['pull_air_vacuum_pump']
            return self.execute_action('niryo_one/commander/robot_action', RobotMoveAction, goal)

        def push_air_vacuum_pump(self, vacuum_pump_id):
            goal = RobotMoveGoal()
            goal.cmd.cmd_type = MoveCommandType.TOOL
            goal.cmd.tool_cmd.tool_id = int(vacuum_pump_id)
            goal.cmd.tool_cmd.cmd_type = self.tool_command_list['push_air_vacuum_pump']
            return self.execute_action('niryo_one/commander/robot_action', RobotMoveAction, goal)
        
        def setup_electromagnet(self, electromagnet_id, pin):
            goal = RobotMoveGoal()
            goal.cmd.cmd_type = MoveCommandType.TOOL
            goal.cmd.tool_cmd.tool_id = int(electromagnet_id)
            goal.cmd.tool_cmd.cmd_type = self.tool_command_list['setup_digital_io']
            goal.cmd.tool_cmd.gpio = pin
            return self.execute_action('niryo_one/commander/robot_action', RobotMoveAction, goal)

        def activate_electromagnet(self, electromagnet_id, pin):
            goal = RobotMoveGoal()
            goal.cmd.cmd_type = MoveCommandType.TOOL
            goal.cmd.tool_cmd.tool_id = int(electromagnet_id)
            goal.cmd.tool_cmd.cmd_type = self.tool_command_list['activate_digital_io']
            goal.cmd.tool_cmd.gpio = pin
            return self.execute_action('niryo_one/commander/robot_action', RobotMoveAction, goal)

        def deactivate_electromagnet(self, electromagnet_id, pin):
            goal = RobotMoveGoal()
            goal.cmd.cmd_type = MoveCommandType.TOOL
            goal.cmd.tool_cmd.tool_id = int(electromagnet_id)
            goal.cmd.tool_cmd.cmd_type = self.tool_command_list['deactivate_digital_io']
            goal.cmd.tool_cmd.gpio = pin
            return self.execute_action('niryo_one/commander/robot_action', RobotMoveAction, goal)

        def get_saved_position_list(self):
            result = self.call_service('niryo_one/position/get_position_list', 
                    GetPositionList, [])
            return result.positions

        def wait(self, time_sleep):
            rospy.sleep(time_sleep)

        def get_joints(self):
            timeout = rospy.get_time() + 2.0
            while self.joints is None:
                rospy.sleep(0.05)
                if rospy.get_time() > timeout:
                    raise NiryoOneException('Timeout: could not get joints (/joint_states topic)')
            return self.joints
    
        def get_arm_pose(self):
            timeout = rospy.get_time() + 2.0
            while self.pose is None:
                rospy.sleep(0.05)
                if rospy.get_time() > timeout:
                    raise NiryoOneException('Timeout: could not get pose (/niryo_one/robot_state topic)')
            return self.pose
        
        def get_hardware_status(self):
            timeout = rospy.get_time() + 2.0
            while self.hw_status is None:
                rospy.sleep(0.05)
                if rospy.get_time() > timeout:
                    raise NiryoOneException('Timeout: could not get hardware status (/niryo_one/hardware_status topic)')
            return self.hw_status

        def get_learning_mode(self):
            timeout = rospy.get_time() + 2.0
            while self.learning_mode_on is None:
                rospy.sleep(0.05)
                if rospy.get_time() > timeout:
                    raise NiryoOneException('Timeout: could not get learning mode (/niryo_one/learning_mode topic)')
            return self.learning_mode_on

        def get_digital_io_state(self):
            timeout = rospy.get_time() + 2.0
            while self.digital_io_state is None:
                rospy.sleep(0.05)
                if rospy.get_time() > timeout:
                    raise NiryoOneException('Timeout: could not get digital io state (/niryo_one/rpi/digital_io_state topic)')
            return self.digital_io_state

        # Will highlight a block on a Blockly interface
        # This is just graphical, no real functionality here
        def highlight_block(self, block_id):
            # rospy.logwarn("Highlight block : " + str(block_id))
            msg = String()
            msg.data = block_id
            self.highlight_block_publisher.publish(msg)

        # Will stop the execution of the current sequence
        # Need to press on "play" button on Niryo One Studio
        # to resume execution
        def break_point(self):
            msg = Int32()
            msg.data = 1
            self.break_point_publisher.publish(msg)
	    # This delay makes sure the program has time
	    # to stop, before executing next command
	    rospy.sleep(0.5)

