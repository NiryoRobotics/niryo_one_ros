#!/usr/bin/env python

#trajectory.py
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
from niryo_one_msgs.msg import TrajectoryPlan 
from std_msgs.msg import Header
from trajectory_msgs.msg import JointTrajectory

class Trajectory: 

    def __init__(self, trajectory_id = '', trajectory_name = "", description = "", group_name = "", 
            joint_trajectory= JointTrajectory() ):
        self.trajectory_id = trajectory_id
        self.trajectory_name = trajectory_name
        self.description = description
        self.group_name = group_name
        self.joint_trajectory = joint_trajectory




