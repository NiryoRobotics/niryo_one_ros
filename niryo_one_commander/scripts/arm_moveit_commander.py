#!/usr/bin/env python

# arm_moveit_commander.py
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
import threading
import moveit_commander

from copy import deepcopy
from math import sqrt
from tf.transformations import quaternion_from_euler

from geometry_msgs.msg import PoseStamped, Pose
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState 
from actionlib_msgs.msg import GoalStatus
from actionlib_msgs.msg import GoalStatusArray
from moveit_msgs.msg import RobotState

from trajectory_msgs.msg import JointTrajectory
from control_msgs.msg import FollowJointTrajectoryActionGoal
from control_msgs.msg import FollowJointTrajectoryActionResult

from niryo_one_commander.robot_commander_exception import RobotCommanderException
from niryo_one_commander.command_status import CommandStatus

TRAJECTORY_TIMEOUT = 15

class ArmMoveitCommander:

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
                raise RobotCommanderException(
                        CommandStatus.PLAN_FAILED, "Moveit failed to compute the plan.")
            else:
                if self.check_trajectory(plan):
                    trajectory_found_but_not_correct = False
                    self.next_plan = plan
                elif plan_counter > 10: # if 10 plans are not enough, then it has a great chance to continue to plan forever
                    rospy.logwarn("Moveit trajectory has been found, but acceleration is not stable.")
                    rospy.logwarn("Max plan tries reach, execute trajectory...")
                    trajectory_found_but_not_correct = False
                    self.next_plan = plan
                else:
                    rospy.logwarn("Moveit trajectory has been found, but acceleration is not stable.")
                    rospy.logwarn("Computing another trajectory...")

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
    
    
    def set_next_plan(self, plan):
        rospy.loginfo("checking if current state is near trajectory start state")
        group_variable_values = self.arm.get_current_joint_values()
        rospy.loginfo("got current joint values:")
        rospy.loginfo("".join(str(x) for x in group_variable_values))
        #print group_variable_values.size()
        #rospy.loginfo(group_variable_values)
        rospy.loginfo("Trajectory start values:")
        rospy.loginfo("".join(str(x) for x in plan.trajectory_start.joint_state.position))
        
        #for idx,val in group_variable_values:
        #    if abs(val-plan.trajectory_start.joint_state.position[idx])>0.01:
        #        rospy.logerr("Start state does not match current state")
        #        return
        #print plan.trajectory_start.type()
        #print plan.trajectory_start.size()
        #rospy.loginfo(plan.trajectory_start)
        rospy.loginfo("Start staet valid, plan set")
        self.next_plan = plan.trajectory
    
    def execute_plan(self, wait=False):
        if self.next_plan:
            
            # reset event
            self.traj_finished_event.clear()
            self.current_goal_id = None
            self.current_goal_result = GoalStatus.LOST

            # send traj and wait 
            self.arm.execute(self.next_plan, wait=False)
            if self.traj_finished_event.wait(TRAJECTORY_TIMEOUT):
                self.previous_plan = self.next_plan
                self.next_plan = None

                if self.current_goal_result == GoalStatus.SUCCEEDED:
                    return CommandStatus.SUCCESS, "Command has been successfully processed"
                elif self.current_goal_result == GoalStatus.PREEMPTED:
                    return CommandStatus.STOPPED, "Command has been successfully stopped"
                elif self.current_goal_result == GoalStatus.ABORTED:
                    # if joint_trajectory_controller aborts the goal, it will still try to 
                    # finish executing the trajectory --> so we ask it to stop from here
                    self.set_position_hold_mode()
                    return CommandStatus.CONTROLLER_PROBLEMS, "Command has been aborted"
                else: # what else could happen ? 
                    return CommandStatus.ROS_ERROR, "Unknown error, try to restart, or contact the support to know more"
            else:
                # todo cancel goal
                self.previous_plan = self.next_plan
                self.next_plan = None
                raise RobotCommanderException(CommandStatus.CONTROLLER_PROBLEMS,
                        "Trajectory timeout - Try to restart the robot")
        else:
            raise RobotCommanderException(CommandStatus.NO_PLAN_AVAILABLE,
                    "You are trying to execute a plan which does't exist")

    # http://wiki.ros.org/joint_trajectory_controller -> preemption policy
    # Send an empty trajectory from the topic interface
    def set_position_hold_mode(self):
        msg = JointTrajectory()
        msg.header.stamp = rospy.Time.now()
        msg.points = []
        rospy.logwarn("SEND POSITION HOLD MODE TO CONTROLLER")
        self.joint_trajectory_publisher.publish(msg)

    def stop_current_plan(self):
        rospy.loginfo("Send STOP to arm")
        self.arm.stop()

    def set_joint_target(self, joint_array):
        self.arm.set_joint_value_target(joint_array)

    def set_position_target(self, x, y, z):
        self.arm.set_position_target([x, y, z], self.end_effector_link)

    def set_rpy_target(self, roll, pitch, yaw):
        self.arm.set_rpy_target([roll, pitch, yaw], self.end_effector_link)
 
    def set_pose_target(self, x, y, z, roll, pitch, yaw):
        self.arm.set_pose_target([x, y, z, roll, pitch, yaw], self.end_effector_link)
        
    def set_pose_quat_target(self,pose_msg):
        self.arm.set_pose_target(pose_msg)

    def set_shift_pose_target(self, axis_number, value):
        self.arm.shift_pose_target(axis_number, value, self.end_effector_link)

    def get_plan_time(self):
        if self.next_plan:
            return self.next_plan.joint_trajectory.points[-1].time_from_start.to_sec()
        else:
            raise RobotCommanderException(CommandStatus.NO_PLAN_AVAILABLE,
                        "No current plan found")
    
    def callback_current_goal(self, msg):
        self.current_goal_id = msg.goal_id.id
        rospy.loginfo("Arm commander - Got a goal id : " + str(self.current_goal_id))

    def callback_goal_result(self, msg):
        if msg.status.goal_id.id == self.current_goal_id:
            #rospy.loginfo("Receive result, goal_id matches.")
            self.current_goal_result = msg.status.status
            rospy.loginfo("Arm commander - Result : " + str(self.current_goal_result))
            self.traj_finished_event.set()
        else:
            rospy.loginfo("Arm commander - Received result, WRONG GOAL ID")


    def __init__(self):

        #moveit_commander.roscpp_initialize(sys.argv)
      
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

        self.traj_finished_event = threading.Event()
        self.current_goal_id = None
        self.current_goal_result = GoalStatus.LOST

        rospy.Subscriber('/niryo_one_follow_joint_trajectory_controller/follow_joint_trajectory/goal',
                FollowJointTrajectoryActionGoal, self.callback_current_goal)

        rospy.Subscriber('/niryo_one_follow_joint_trajectory_controller/follow_joint_trajectory/result',
                FollowJointTrajectoryActionResult, self.callback_goal_result)

        # Direct topic to joint_trajectory_controller
        # Used ONLY when goal is aborted, to enter position hold mode
        self.joint_trajectory_publisher = rospy.Publisher(
                '/niryo_one_follow_joint_trajectory_controller/command',
                JointTrajectory, queue_size=10)


