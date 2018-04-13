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
        with self.lock: 
            with open(self.base_dir + filename, 'w') as f:
                f.write("position name: \n") 
                f.write(str(position.position_name)+"\n") 
                f.write("prefixe: \n") 
                f.write( str(position.prefixe)+"\n") 
                f.write("position ID: \n") 
                f.write(str(position.position_id)+"\n") 
                f.write("Joints: \n")
                f.write(str(position.joints).strip('[]')+"\n")
                f.write( "Pose:  \n") 
                f.write( str(position.pose).strip('[]')+"\n") 

    def does_file_exist(self, filename):
        filenames = self.get_all_filenames()
        return filename in filename
    def get_all_filenames(self):
        filenames = []
        try:
            filenames = os.listdir(self.base_dir)
        except OSError:
            pass
        r = re.compile("^position_\d+$")
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
                    if line.startswith('position name: '):
                        pos.position_name = str(next(f).rstrip())
                    if line.startswith("prefixe:"): 
                        pos.prefixe=str(next(f).rstrip())
                    if line.startswith("position ID:"):
                        pos.position_id=int(next(f).rstrip())
                    if line.startswith("Joints: "): 
                        pos.joints=list(str(next(f).rstrip()).split(','))
                        pos.joints=map(int,pos.joints)
                    if line.startswith("Pose:"):
                        pos.pose=list(str(next(f).rstrip()).split(','))
                        pos.pose=map(int,pos.pose)
                return pos 



    def remove_position(self, position_name):
        filename = self.filename_from_position_name(position_name)
        with self.lock:
            try:
                os.remove(self.base_dir + filename)
            except OSError as e:
                raise NiryoOneFileException("Could not remove sequence with id "+position_name + " : " + str(e))


if __name__ == '__main__': 
    fh=PositionFileHandler("~/niryo_one_position")   # create a dir niryo_one_position
    print(fh.base_dir)
    p=Position("move", "pl1", 1, [1,2,3,4,5,6], [1,2,3,4,5,6]) 
    fh.write_position(p)
    pos=fh.read_position("move")
    print(pos.joints)
    print(pos.pose)
    joint=pos.joints[1]
    print(joint)
    fh.remove_position("move")
    







