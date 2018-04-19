#!/usr/bin/env python

#position_file_handler.py
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

import os
import re
from threading import Lock

from niryo_one_commander.position.moveit_utils import get_forward_kinematic
from niryo_one_commander.position.position import Position 
from niryo_one_user_interface.sequences.niryo_one_file_exception import NiryoOneFileException

class PositionFileHandler:
    def __init__(self, position_dir):
        self.base_dir = position_dir
        if self.base_dir.startswith('~'):
            self.base_dir = os.path.expanduser(self.base_dir)
        if not self.base_dir.endswith('/'):
            self.base_dir += '/'
        if not os.path.exists(self.base_dir): 
            print("Create positions dir " + str(self.base_dir))
            os.makedirs(self.base_dir)
        self.lock = Lock()
    
    def position_name_from_filename(self, filename):
        return int(filename.replace('position_', ''))

    def filename_from_position_name(self, position_name):
        return 'position_' + position_name
    
    def write_position(self, position ): 
        filename = self.filename_from_position_name(position.position_name)
        (position.point, position.rpy, position.quaternion) =  get_forward_kinematic(position.joints)
        with self.lock: 
            with open(self.base_dir + filename, 'w') as f:
                f.write("Position_Name:\n") 
                f.write(str(position.position_name)+"\n") 
                f.write("Position_Id:\n") 
                f.write(str(position.position_id)+"\n") 
                f.write("Joints:\n")
                f.write(str(position.joints).strip('()')+"\n") 
                f.write( "RPY:\n") 
                f.write( str(position.rpy.roll)+ "\n") 
                f.write( str(position.rpy.pitch)+ "\n") 
                f.write( str(position.rpy.yaw)+ "\n") 
                f.write( "Point:\n") 
                f.write( str(position.point.x)+ "\n") 
                f.write( str(position.point.y)+ "\n") 
                f.write( str(position.point.z)+ "\n")
                f.write( "Quaternion:\n") 
                f.write( str(position.quaternion.x)+ "\n") 
                f.write( str(position.quaternion.y)+ "\n") 
                f.write( str(position.quaternion.z)+ "\n") 
                f.write( str(position.quaternion.w)+ "\n")


    def does_file_exist(self, filename):
        filenames = self.get_all_filenames()
        return filename in filenames
    def get_all_filenames(self):
        filenames = []
        try:
            filenames = os.listdir(self.base_dir)
        except OSError:
            pass
        r = re.compile("^position_.+$")
        # Keep only correct filenames
        return filter(r.match, filenames)



    def read_position(self,position_name):
        filename = self.filename_from_position_name(position_name)
        # Check if exists
        if not self.does_file_exist(filename):
            raise NiryoOneFileException(' ' + str(position_name)+ ' does not exist')
        with self.lock:
            with open(self.base_dir + filename, 'r') as f:
                pos = Position()
                for line in f:
                    if line.startswith('Position_Name:'):
                        pos.position_name = str(next(f).rstrip())
                    if line.startswith("Position_Id:"):
                        pos.position_id = int(next(f).rstrip())
                    if line.startswith("Joints:"): 
                        pos.joints = list(str(next(f).rstrip()).split(','))
                        pos.joints = map(float ,pos.joints)
                    if line.startswith("RPY:"):
                        pos.rpy.roll = float(str(next(f).rstrip()))
                        pos.rpy.pitch = float(str(next(f).rstrip()))
                        pos.rpy.yaw = float(str(next(f).rstrip()))
                    if line.startswith("Point:"):
                        pos.point.x = float(str(next(f).rstrip()))
                        pos.point.y = float(str(next(f).rstrip()))
                        pos.point.z = float(str(next(f).rstrip())) 
                    
                    if line.startswith("Quaternion:"):
                        pos.quaternion.x = float(str(next(f).rstrip()))
                        pos.quaternion.y = float(str(next(f).rstrip()))
                        pos.quaternion.z = float(str(next(f).rstrip())) 
                        pos.quaternion.w = float(str(next(f).rstrip())) 


                return pos 



    def remove_position(self, position_name):
        filename = self.filename_from_position_name(position_name)
        with self.lock:
            try:
                os.remove(self.base_dir + filename)
            except OSError as e:
                raise NiryoOneFileException("Could not remove sequence with id " + position_name + " : " + str(e))
    
    

        # choose a non used, incremental id
    def pick_new_id(self):
        filenames = self.get_all_filenames()
        max_id = 0
        for filename in filenames:
            current_id = self.position_id_from_filename(filename)
            if current_id > max_id:
                max_id = current_id
        return max_id + 1

    def position_id_from_filename(self, filename):
        with open(self.base_dir + filename, 'r') as f:
            for line in f:
                if line.startswith("Position_Id:"):
                    position_id = int(next(f).rstrip())
        return position_id





if __name__ == '__main__': 
    pass 





