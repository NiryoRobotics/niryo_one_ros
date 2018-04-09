#!/usr/bin/env python

# matlab_manager.py
# Copyright (C) 2018 Niryo
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
from niryo_one_msgs.msg import * 

#Messages
from niryo_one_msgs.msg import RobotMoveCommand
from niryo_one_msgs.msg import MatlabMoveResult 

#msgs action
from niryo_one_msgs.msg import RobotMoveAction
from niryo_one_msgs.msg import RobotMoveGoal
from niryo_one_msgs.msg import RobotMoveResult
"""
This class is the main interface between Mtalab and niryo_one 
it's create a publisher niryo_one_matlab/command to publish joints goal from matlab interface 
those goals are sent by the action server client created 
after executing the command niryo_one_matlab/result publisher publish MatlabMoveResult message  
"""
class MatlabManager:
   
    def create_response(self, status, message):
        return {'status': status, 'message': message}
    
    def callback_matlab_node(self,request):
        rospy.loginfo("........................matlab node ........................................")
        cmd=request.joints
        cmd_type=request.cmd_type
        status=1
        message="Sueccess"
        response=self.send_matlab_goal(cmd,cmd_type)
        msg=MatlabMoveResult()
        rospy.loginfo("Matlab node publisher......")
        msg.status=response.status
        msg.message=response.message
        rospy.loginfo(msg)
        rospy.sleep(0.2)
        self.matlab_node_publisher.publish(msg)
        rospy.loginfo("Matlab Move Result Published")

    def __init__(self): 
        self.matlab_node=rospy.Subscriber('/niryo_one_matlab/command',RobotMoveCommand,self.callback_matlab_node)  
        self.matlab_node_publisher=rospy.Publisher('/niryo_one_matlab/result',MatlabMoveResult,queue_size=10) 
        rospy.loginfo('----------Matlab node -------------------------') 
    
    def send_matlab_command(self,commande):
        self.action_client_matlab.send_goal(commande)
        self.action_client_matlab.wait_for_result()
        return  self.action_client_matlab.get_result()
    
    def send_matlab_goal(self,cmd,cmd_type):    
        self.action_client_matlab= actionlib.SimpleActionClient('/niryo_one/commander/robot_action',niryo_one_msgs.msg.RobotMoveAction)
        rospy.loginfo("waitnig for action server: niryo_one/robot_action....")
        self.action_client_matlab.wait_for_server()
        rospy.loginfo("Found action server : niryo_one/robot_action")
        goal=RobotMoveGoal()
        goal.cmd.joints=cmd
        goal.cmd.cmd_type=cmd_type
        self.action_client_matlab.send_goal(goal)
        rospy.loginfo("waiting for result")
        self.action_client_matlab.wait_for_result()
        rospy.loginfo("result........")
        response= self.action_client_matlab.get_result()
        result = self.create_response(response.status,response.message)
        return response


