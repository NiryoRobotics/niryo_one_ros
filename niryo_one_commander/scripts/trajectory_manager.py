#!/usr/bin/env python

# trajectory_manager.py 
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
from niryo_one_user_interface.sequences.niryo_one_file_exception import NiryoOneFileException
from niryo_one_commander.trajectory.trajectory import Trajectory
from niryo_one_commander.trajectory.trajectory_command_type import TrajectoryCommandType
from niryo_one_commander.trajectory.trajectory_file_handler import TrajectoryFileHandler
from niryo_one_msgs.msg import TrajectoryPlan 
from trajectory_msgs.msg import JointTrajectory 
from niryo_one_msgs.srv import ManageTrajectory
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectoryPoint


class TrajectoryManager:
    
    def __init__(self, trajectory_dir): 
        self.fh = TrajectoryFileHandler(trajectory_dir)
        self.manage_position_server = rospy.Service('/niryo_one/trajectory/manage_trajectory', ManageTrajectory, self.callback_manage_trajectory)
        rospy.loginfo("/niryo_one/position/manage_position service has been created " )


    def create_trajectory_response(self, status, message, trajectory=None):
        trajectory_msg = TrajectoryPlan()
        if trajectory != None:
            trajectory_msg.group_name = trajectory.group_name
            trajectory_msg.trajectory = trajectory.joint_trajectory
            trajectory_id = trajectory.trajectory_id
            trajectory_name = trajectory.trajectory_name
            description = trajectory.description

        return { 'status': status, 'message': message, 'trajectory_id': trajectory_id,
            'trajectory_name':  trajectory_name, 'description': description,'trajectory_plan': trajectory_msg }

    def callback_manage_trajectory(self, req):
        cmd_type = req.cmd_type
        trajectory_id = req.trajectory_id
        '''trajectory_name = req.trajectory_name
        description = req.description'''
        trajectory_msg = req.trajectory_plan
        trajectory_data = Trajectory( trajectory_id = req.trajectory_id, trajectory_name = req.trajectory_name, description = req.description, group_name = req.trajectory_plan.group_name, 
            joint_trajectory = req.trajectory_plan.trajectory )
         
         # GET an existing trajectory 
        if cmd_type == TrajectoryCommandType.GET:
            traj = self.get_trajectory(trajectory_id)
            if traj == None:
                return self.create_trajectory_response(400, "No trajectory found with id : " + trajectory_id)
            return self.create_trajectory_response(200, "trajectory has been found", traj)

    



    def get_trajectory(self, trajectory_id):
        try:	
            return self.fh.read_trajectory_with_json(trajectory_id)
        except NiryoOneFileException as e:
            return None
    





    def client(self, traj):
        rospy.wait_for_service('/niryo_one/trajectory/manage_trajectory')
        client_service = rospy.ServiceProxy('/niryo_one/trajectory/manage_trajectory', ManageTrajectory)
        trajectory_msg = TrajectoryPlan()
        resp1 = client_service(1,4,traj.trajectory_id,traj.trajectory_name,traj.description,trajectory_msg)

        print (resp1)
        return(resp1)

if __name__ == '__main__': 
    tm = TrajectoryManager('/home/sarra/trajectories')
    traj=Trajectory()
    tm.client(traj)
# this for test 
    '''plan= JointTrajectory()
    group_name='arm'
    plan.joint_names=['joint_1','joint_2', 'joint_3', 'joint_4','joint_5', 'joint_6']
    plan.header=Header(0,2,"/ground_link")
    tj=JointTrajectoryPoint()
    tj.positions=[1,1,1,1,1]
    point = JointTrajectoryPoint()
    point.positions = [0.0,0.0,1.545, 0.33, -1.57, -0.486, 0.0, 0.0]     
    plan.points = [point,tj]
    traj = Trajectory(trajectory_id =4, trajectory_name="sarra", description="this is a test",group_name=group_name,  joint_trajectory=plan )
    fh = TrajectoryFileHandler('/home/sarra/trajectories')'''
    #print traj.__dict__
    #print "------"
    #fh.write_trajectory(traj)
    #traje=fh.read_trajectory(4)
    #print(traje)
    #fh.write_trajectroy_with_json(traj)
    #trajectory=fh.read_trajectory_with_json(4)
    #print trajectory.joint_trajectory.points[0].positions