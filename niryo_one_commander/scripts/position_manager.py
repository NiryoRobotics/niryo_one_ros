#!/usr/bin/env python

# position_manager.py 
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



import rospy
from niryo_one_commander.position.position  import Position 
from niryo_one_commander.position.niryo_one_file_exception import NiryoOneFileException
from niryo_one_commander.position.position_file_handler import PositionFileHandler
from niryo_one_commander.position.position_command_type import PositionCommandType
from niryo_one_commander.moveit_utils import get_forward_kinematic

from niryo_one_msgs.msg import Position  as PositionMessage 
from niryo_one_msgs.srv import ManagePosition 
from niryo_one_msgs.msg import RPY
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion

class PositionManager:
   
    def __init__(self, position_dir):
        self.fh = PositionFileHandler(position_dir)
        self.manage_position_server = rospy.Service('/niryo_one/position/manage_position', ManagePosition, self.callback_manage_position)
        rospy.loginfo("service manage position created") 
        
    def create_position_response(self, status, message, position=None):
        position_msg = PositionMessage()
        if position != None:
            position_msg.position_name = position.position_name
            position_msg.position_id = position.position_id
            position_msg.joints = position.joints 
            position_msg.rpy = Position.RPY(position.rpy.roll, position.rpy.pitch, position.rpy.yaw)
            position_msg.point = Position.Point( position.point.x, position.point.y, position.point.z) 
            position_msg.quaternion = Position.Quaternion(position.quaternion.x, position.quaternion.y,
		 position.quaternion.z, position.quaternion.w)
        return { 'status': status, 'message': message, 'position': position_msg }
    
    def callback_manage_position(self, req):
        cmd_type = req.cmd_type
        position_name = req.position_name 
        position_msg = req.position 
	rpy = Position.RPY(position_msg.rpy.roll, position_msg.rpy.pitch,  position_msg.rpy.yaw)
        point = Position.Point(position_msg.point.x, position_msg.point.y, position_msg.point.z)
        quaternion = Position.Quaternion(position_msg.quaternion.x, position_msg.quaternion.y, position_msg.quaternion.z,
		  position_msg.quaternion.w )
        position_data = Position(position_name = position_msg.position_name, position_id = position_msg.position_id, 
		joints = position_msg.joints  , rpy=rpy, point = point, quaternion =  quaternion)     
        # GET an existing position 
        if cmd_type == PositionCommandType.GET:
            pos = self.get_position(position_name)
            if pos == None:
                return self.create_position_response(400, "No position found with id : " + position_name)
            return self.create_position_response(200, "position has been found", pos)
    
    # CREATE new position    
        elif cmd_type == PositionCommandType.CREATE:
            new_position_name = self.create_new_position(position_data)
            new_position = self.get_position(new_position_name)
            if new_position == None:
                return self.create_position_response(400, "Failed to create position")
            return self.create_position_response(200, "position has been created", new_position)
    
    # UPDATE existing sequence
        elif cmd_type == PositionCommandType.UPDATE:
            pos = self.get_position(position_name)
            if pos == None:
                return self.create_position_response(400, "No position found with this name : " + position_name)
            success = self.update_position(pos, position_data)
            if not success:
                return self.create_position_response(400, "Could not update position with this name : " + position_name)
            return self.create_position_response(200, "Position has been updated", pos)
    
       # DELETE sequence
        elif cmd_type == PositionCommandType.DELETE:
            success = self.delete_position(position_name)
            if not success:
                return self.create_position_response(400, "Could not delete position with name : " + position_name)
            return self.create_position_response(200, "Position  has been deleted")
    
    def delete_position(self, position_name):
        try:
            self.fh.remove_position(position_name)
        except NiryoOneFileException as e:
            return False
        return True
    
    def update_position(self, position, position_data):
        position.position_name = position_data.position_name
        position.joints = position_data.joints
	position.position_id = position.position_id
        (position.point, position.rpy, position.quaternion) = get_forward_kinematic(position.joints)
	
                                
        try:
            self.fh.write_position(position)
        except NiryoOneFileException as e:
            return False
        return True

    def get_position(self, position_name):
        try:	
            return self.fh.read_position(position_name)
        except NiryoOneFileException as e:
            return None

    def create_new_position(self, position) : 
        try:
	    position.position_id =  self.fh.pick_new_id()
	    (position.point, position.rpy, position.quaternion) = get_forward_kinematic(position.joints)    
            self.fh.write_position(position)
            return(position.position_name)
        except  NiryoOneFileException as e:
            return None

if __name__ == '__main__':
    pass 
    


