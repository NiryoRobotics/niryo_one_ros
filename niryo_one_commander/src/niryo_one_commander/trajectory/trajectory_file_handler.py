#!/usr/bin/env python

#trajectory_file_handler.py
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
from niryo_one_user_interface.sequences.niryo_one_file_exception import NiryoOneFileException
from niryo_one_commander.trajectory.trajectory import Trajectory
import jsonpickle


from trajectory_msgs.msg import JointTrajectory 
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectoryPoint 


class TrajectoryFileHandler:
       
    def __init__(self, trajectory_dir):
        self.base_dir = trajectory_dir
        if self.base_dir.startswith('~'):
            self.base_dir = os.path.expanduser(self.base_dir)
        if not self.base_dir.endswith('/'):
            self.base_dir += '/'
        if not os.path.exists(self.base_dir): 
            print("Create trajectory dir " + str(self.base_dir))
            os.makedirs(self.base_dir)
        self.lock = Lock()
    def get_all_filenames(self):
        filenames = []
        try:
            filenames = os.listdir(self.base_dir)
        except OSError:
            pass
        r = re.compile("^trajectory_\d+$")
        # Keep only correct filenames
        return filter(r.match, filenames)

    def does_file_exist(self, filename):
        filenames = self.get_all_filenames()
        return filename in filenames

    def trajectory_id_from_filename(self, filename):
        return int(filename.replace('trajectory_', '')) 

    def filename_from_trajectory_id(self, traj_id):
        return 'trajectory_' + str(traj_id)

    def remove_trajectory(self, traj_id):
        filename = self.filename_from_trajectory_id(traj_id)
        with self.lock:
            try:
                os.remove(self.base_dir + filename)
            except OSError as e:
                raise NiryoOneFileException("Could not remove trajectory with id " 
                        + str(traj_id) + " : " + str(e))

    def pick_new_id(self):
        filenames = self.get_all_filenames()
        max_id = 0
        for filename in filenames:
            current_id = self.rajectory_id_from_filename(filename)
            if current_id > max_id:
                max_id = current_id
        return max_id + 1
                     
    def write_trajectroy_with_json(self,traj):
        filename = self.filename_from_trajectory_id(traj.trajectory_id)
        with self.lock: 
            with open(self.base_dir + filename, 'w') as f:
                json_obj = jsonpickle.encode(traj)
                f.write(json_obj)
                
    def read_trajectory_with_json(self,trajectory_id ): 
        filename = self.filename_from_trajectory_id(trajectory_id)
        # Check if exists
        if not self.does_file_exist(filename):
            raise NiryoOneFileException(' ' + str(trajectory_id)+ ' does not exist')
        with self.lock:
            with open(self.base_dir + filename, 'r') as f:
                json_str = f.read()
                traj = jsonpickle.decode(json_str)
                return traj


if __name__ == '__main__': 

# this for test 
    plan= JointTrajectory()
    group_name='arm'
    plan.joint_names=['joint_1','joint_2', 'joint_3', 'joint_4','joint_5', 'joint_6']
    plan.header=Header(0,2,"/ground_link")
    tj=JointTrajectoryPoint()
    tj.positions=[1,1,1,1,1]
    point = JointTrajectoryPoint()
    point.positions = [0.0,0.0,1.545, 0.33, -1.57, -0.486, 0.0, 0.0]     
    plan.points = [point,tj]
    traj = Trajectory(trajectory_id =4, trajectory_name="sarra", description="this is a test",group_name=group_name,  joint_trajectory=plan )
    fh = TrajectoryFileHandler('/home/sarra/trajectories')
    #print traj.__dict__
    #print "------"
    #fh.write_trajectory(traj)
    #traje=fh.read_trajectory(4)
    #print(traje)
    fh.write_trajectroy_with_json(traj)
    trajetory=fh.read_trajectory_with_json(4)
    print trajetory.description
    #a=traje['trajectory_name']
    #print a 
    #prinht(traje.joint_trajectory.points[0].positions)
    #print(traje.joint_trajectory.points[1])
    
