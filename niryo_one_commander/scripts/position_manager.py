#!/usr/bin/env python

# position_manager.py 
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

from niryo_one_msgs.msg import Position  as PositionMessage 
from niryo_one_msgs.srv import ManagePosition 

from niryo_one_commander.position.position  import Position 
from niryo_one_user_interface.sequences.niryo_one_file_exception import NiryoOneFileException
from niryo_one_commander.position.position_file_handler import PositionFileHandler
from niryo_one_commander.position.position_command_type import PositionCommandType

class PositionManager:
   
    def __init__(self, position_dir):
        self.fh = PositionFileHandler(position_dir)
        self.manage_position_server = rospy.Service('/niryo_one/position/manage_position', ManagePosition, self.callback_manage_position)
        rospy.loginfo("service manage position created") 
        self.client()
   
    def create_position_response(self, status, message, position=None):
        position_msg = PositionMessage()
        if position != None:
            position_msg.position_name = position.position_name
            position_msg.prefixe = position.prefixe 
            position_msg.position_id=position.position_id 
            position_msg.joints=position.joints 
            position_msg.pose= position.pose
        return { 'status': status, 'message': message, 'position': position_msg }
    
    def callback_manage_position(self, req):
        cmd_type = req.cmd_type
        position_name = req.position_name 
        position_msg = req.position 
        position_data = Position(position_name=position_msg.position_name, prefixe=position_msg.prefixe, position_id=position_msg.position_id, joints=position_msg.joints , pose=position_msg.pose )

      # GET an existing position 
        if cmd_type == PositionCommandType.GET:
            pos = self.get_position(position_name)
            if pos == None:
                return self.create_position_response(400, "No position found with id : " + position_name)
            return self.create_position_response(200, "position has been found", pos)
    
    # CREATE new position    
        elif cmd_type == PositionCommandType.CREATE:
            new_position_name= self.create_new_position(position_data)
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
            return self.create_position_response(200, "Sequence has been deleted")

       # GET last executed
        elif cmd_type == PositionCommandType.GET_LAST_EXECUTED:
            pos = self.get_last_executed_position()
            if pos == None:
                return self.create_position_response(400, "No last executed position has been found")
            return self.create_position_response(200, " position has been found", seq)
    
    def delete_position(self, position_name):
        try:
            self.fh.remove_position(position_name)
        except NiryoOneFileException as e:
            return False
        return True
     
    def get_last_executed_position(self):
        return self.get_position("last_executed_position")


    def save_last_executed_sequence(self, position):
        position.position_name="last_executed_postion"
        try:
            self.fh.write_position(position)
        except NiryoOneFileException as e:
            return -1
        return 0


    def update_position(self, position, position_data):
        position.position_name=position_data.position_name
        position.prefixe=position_data.prefixe 
        position.position_id=position_data.position_id
        position.joints=position_data.joints
        position.pose=position_data.pose 

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

    def create_new_position(self,position) : 
        try: 
            self.fh.write_position(position)
            return(position.position_name)
        except  NiryoOneFileException as e:
            return None

    def client (self): 
        rospy.wait_for_service('/niryo_one/position/manage_position')
        client_service = rospy.ServiceProxy('/niryo_one/position/manage_position', ManagePosition)
      
    
       
        p=Position()
        resp1 = client_service(4,"new", p)
        print (resp1)

if __name__ == '__main__':


    rospy.init_node('niryo_one_position_manager') 
    pm=PositionManager("~/niryo_one_position") 
    rospy.spin()



