#!/usr/bin/env python

# arm_commander.py
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
from niryo_one_commander.move_group_arm import MoveGroupArm

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

class ArmCommander:

    def set_next_plan(self, traj):
        next_plan = traj.trajectory
        return(next_plan)
    
    def execute_plan(self, next_plan , wait=False):
        if next_plan:
            # reset event
            self.traj_finished_event.clear()
            self.current_goal_id = None
            self.current_goal_result = GoalStatus.LOST

            # send traj and wait 
            self.move_group_arm.arm.execute(next_plan, wait=False)
            if self.traj_finished_event.wait(TRAJECTORY_TIMEOUT):
                previous_plan = next_plan
                next_plan = None

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
                previous_plan = next_plan
                next_plan = None
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
        self.move_group_arm.arm.stop()

    def set_joint_target(self, joint_array):
        self.move_group_arm.arm.set_joint_value_target(joint_array)

    def set_position_target(self, x, y, z):
        self.move_group_arm.arm.set_position_target([x, y, z], self.move_group_arm.end_effector_link)

    def set_rpy_target(self, roll, pitch, yaw):
        self.move_group_arm.arm.set_rpy_target([roll, pitch, yaw], self.move_group_arm.end_effector_link)
 
    def set_pose_target(self, x, y, z, roll, pitch, yaw):
        self.move_group_arm.arm.set_pose_target([x, y, z, roll, pitch, yaw], self.move_group_arm.end_effector_link)
        
    def set_pose_quat_target(self,pose_msg):
        self.move_group_arm.arm.set_pose_target(pose_msg)

    def set_shift_pose_target(self, axis_number, value):
        self.move_group_arm.arm.shift_pose_target(axis_number, value, self.move_group_arm.end_effector_link)

    def get_plan_time(self, next_plan):
        if next_plan:
            return next_plan.joint_trajectory.points[-1].time_from_start.to_sec()
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


    def __init__(self,move_group_arm):
        self.move_group_arm = move_group_arm 
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


