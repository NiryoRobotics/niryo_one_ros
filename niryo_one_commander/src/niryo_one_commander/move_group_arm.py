#!/usr/bin/env python

# move_group_arm.py
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
import moveit_commander


class MoveGroupArm: 
   
    def __init__(self): 
        # Get params from rosparams
        reference_frame            = rospy.get_param("~reference_frame")
        move_group_commander_name  = rospy.get_param("~move_group_commander_name")
        allow_replanning           = rospy.get_param("~allow_replanning")
        goal_joint_tolerance       = rospy.get_param("~goal_joint_tolerance")
        goal_position_tolerance    = rospy.get_param("~goal_position_tolerance")
        goal_orientation_tolerance = rospy.get_param("~goal_orientation_tolerance")

        # Set reference_frame
        self.reference_frame = reference_frame
        
        # Get Arm MoveGroupCommander
        move_group_arm_ok = False
        while (not move_group_arm_ok):
            try:
                rospy.loginfo("Trying to get 'arm' group from moveit...")
                self.arm = moveit_commander.MoveGroupCommander(move_group_commander_name)
                move_group_arm_ok = True
            except RuntimeError as e:
                rospy.loginfo(e)
                rospy.sleep(1.0)       
    
        # Get end effector link
        self.end_effector_link = self.arm.get_end_effector_link()
        
        # Set pose reference frame
        self.arm.set_pose_reference_frame(self.reference_frame)
       
        # Set planning parameters
        self.arm.allow_replanning(allow_replanning)
        self.arm.set_goal_joint_tolerance(goal_joint_tolerance)
        self.arm.set_goal_position_tolerance(goal_position_tolerance)
        self.arm.set_goal_orientation_tolerance(goal_orientation_tolerance)

        rospy.loginfo("Successfully connected to move_group." +
                "\n" + "Started group     : " + str(self.arm.get_name()) + 
                "\n" + "Planning_frame    : " + str(self.arm.get_planning_frame()) + 
                "\n" + "Reference frame   : " + str(self.reference_frame) + 
                "\n" + "End effector link : " + str(self.end_effector_link))
        
        rospy.loginfo("Arm Moveit Commander has been started")

    """
    MoveitCommander documentation :
    http://docs.ros.org/indigo/api/moveit_commander/html/classmoveit__commander_1_1move__group_1_1MoveGroupCommander.html#a63bbf526cd62983ca44536ed6dd16813

    """
    
    def compute_plan(self):
        trajectory_found_but_not_correct = True
        plan_counter = 0

        while trajectory_found_but_not_correct:
            plan = self.arm.plan()
            plan_counter += 1

            if not plan.joint_trajectory.points:
                return None 
            else:
                if self.check_trajectory(plan):
                    trajectory_found_but_not_correct = False
                    next_plan = plan
                elif plan_counter > 10: # if 10 plans are not enough, then it has a great chance to continue to plan forever
                    rospy.logwarn("Moveit trajectory has been found, but acceleration is not stable.")
                    rospy.logwarn("Max plan tries reach, execute trajectory...")
                    trajectory_found_but_not_correct = False
                    next_plan = plan
                else:
                    rospy.logwarn("Moveit trajectory has been found, but acceleration is not stable.")
                    rospy.logwarn("Computing another trajectory...")
        return(next_plan)

    # see --> https://github.com/ros-planning/moveit/issues/416
    # Sometimes (on Kinetic) the trajectory will slow down, one or multiple
    # times, at any moment. Here we check if there are some variations in 
    # traj accelerations. If no, it means for each axis, vel is going up,
    # then down (correct case). If yes, we should retry to compute the traj
    
    def check_trajectory(self, plan):
        for i in range(0,6):
            accs = []
            for point in plan.joint_trajectory.points:
                accs.append(point.accelerations[i]) 

            sign_change_counter = 0
            last_acc = accs[1] # accs[0] is 0.0

            for acc in accs:
                if (acc == 0.0):
                    pass
                else:
                    if (acc < 0.0 and last_acc > 0.0) or (acc > 0.0 and last_acc < 0.0):
                        sign_change_counter += 1
                    last_acc = acc

            if sign_change_counter > 1:
                return False
            #rospy.loginfo("Sign change counter : " + str(sign_change_counter))
        return True

    def execute(self, plan, wait=False):
        self.arm.execute(plan, wait=wait)

    def stop(self):
        self.arm.stop()

    def set_joint_value_target(self, joint_array):
        self.arm.set_joint_value_target(joint_array)

    def set_position_target(self, x, y, z):
        self.arm.set_position_target([x, y, z], self.end_effector_link)

    def set_rpy_target(self, roll, pitch, yaw):
        self.arm.set_rpy_target([roll, pitch, yaw], self.end_effector_link)
 
    def set_pose_target(self, x, y, z, roll, pitch, yaw):
        self.arm.set_pose_target([x, y, z, roll, pitch, yaw], self.end_effector_link)
        
    def set_pose_quat_target(self, pose):
        self.arm.set_pose_target(pose, self.end_effector_link)

    def set_shift_pose_target(self, axis_number, value):
        self.arm.shift_pose_target(axis_number, value, self.end_effector_link)

    def set_max_velocity_scaling_factor(self, percentage):
        self.arm.set_max_velocity_scaling_factor(percentage)

