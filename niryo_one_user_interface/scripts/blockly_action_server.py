#!/usr/bin/env python

# blockly_action_server.py
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

# import Niryo api
from niryo_one_python_api.niryo_one_api import *

# Action msgs
from niryo_one_msgs.msg import BlocklyAction
from niryo_one_msgs.msg import BlocklyGoal
from niryo_one_msgs.msg import BlocklyResult

from niryo_one_commander.command_status import CommandStatus

from blockly_code_generator import *

class BlocklyActionServer:

    def __init__(self):
        self.blockly_generator = BlocklyCodeGenerator()
    
        self.server = actionlib.ActionServer('niryo_one/blockly_action',
                BlocklyAction, self.on_goal, self.on_cancel, auto_start=False)

        self.current_goal_handle = None

    def shutdown(self):
        self.blockly_generator.shutdown()

    def start(self):
        self.server.start()
        rospy.loginfo("Action Server started (Blockly Action)")

    def create_result(self, status, message):
        result = BlocklyResult()
        result.status = status
        result.message = message
        return result

    def on_goal(self, goal_handle):
        rospy.loginfo("Blockly action server : Received goal. Check if exists")
        # print goal_handle.__dict__

        # check if still have a goal -> set_rejected() 
        if self.current_goal_handle is not None:
            result = self.create_result(CommandStatus.GOAL_STILL_ACTIVE, "Current command still active. Cancel it if you want to execute a new one")
            goal_handle.set_rejected(result)
            return
      
        # set accepted
        self.current_goal_handle = goal_handle
        self.current_goal_handle.set_accepted()
        rospy.loginfo("Blockly : Goal has been accepted")

        # Launch compute + execution in a new thread
        w = threading.Thread(name="worker", target=self.execute_command)
        w.start()
        rospy.loginfo("Blockly : Executing thread")

    def on_cancel(self, goal_handle):
        rospy.loginfo("Received cancel command")

        if goal_handle == self.current_goal_handle:
            self.cancel_current_command()
        else:
            rospy.loginfo("No current goal, nothing to do")

    def cancel_current_command(self):
        pass
        # for now, a quick way to terminate a blockly command is to activate learning mode

    def execute_command(self):
        cmd = self.current_goal_handle.goal.goal.cmd
        xml = cmd.xml

        rospy.loginfo("Generate Python code from xml")
        response = self.blockly_generator.get_generated_python_code(xml)
        if response['status'] != 200:
            result = self.create_result(CommandStatus.BLOCKLY_FAILED, str(response['message']))
            self.current_goal_handle.set_aborted(result)
            self.current_goal_handle = None
            return

        code = response['code']
        rospy.loginfo("About to execute code...")
        rospy.loginfo(code)
            
        try:
            exec code
        except NiryoOneException, e:
            result = self.create_result(CommandStatus.BLOCKLY_FAILED, str(e)) 
            self.current_goal_handle.set_aborted(result)
            self.current_goal_handle = None
            return
        except Exception, e:
            result = self.create_result(CommandStatus.BLOCKLY_FAILED, str(e)) 
            self.current_goal_handle.set_aborted(result)
            self.current_goal_handle = None
            return
    
        result = self.create_result(CommandStatus.SUCCESS, "Successfully executed Blockly code")
        # print result
        self.current_goal_handle.set_succeeded(result)
        self.current_goal_handle = None
    

if __name__ == '__main__':
    #rospy.init_node('blockly_action_server')
    #server = BlocklyActionServer()
    #server.start()
    #rospy.spin()
    pass
