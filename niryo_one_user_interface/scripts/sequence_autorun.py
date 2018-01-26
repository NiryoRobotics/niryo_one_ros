#!/usr/bin/env python

# sequence_autorun.py
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
import actionlib
import os
from threading import Lock
from threading import Thread

from std_msgs.msg import Bool
from niryo_one_msgs.msg import HardwareStatus
from niryo_one_msgs.msg import SequenceAutorunStatus
from niryo_one_msgs.srv import SetInt
from niryo_one_msgs.srv import SetSequenceAutorun

# Action msgs
from niryo_one_msgs.msg import SequenceAction
from niryo_one_msgs.msg import SequenceGoal
from niryo_one_msgs.msg import SequenceResult

from niryo_one_msgs.msg import RobotMoveAction
from niryo_one_msgs.msg import RobotMoveGoal

from niryo_one_user_interface.sequences.sequence import Sequence
from niryo_one_user_interface.sequences.sequence_action_type import SequenceActionType
from niryo_one_commander.command_type import CommandType as MoveCommandType

class SequenceAutorunMode:
    LOOP = 0
    ONE_SHOT = 1

class SequenceAutorun:

    def read_sequence_autorun_status(self):
        enabled = False;
        mode = SequenceAutorunMode.LOOP
        sequence_ids = []
        if os.path.isfile(self.sequence_autorun_status_file):
            with self.lock:
                with open(self.sequence_autorun_status_file, 'r') as f:
                    for line in f:
                        if line.startswith('enabled:'):
                            enabled = (line.replace('enabled:','').rstrip() == 'true')
                        if line.startswith('mode:'):
                            mode = int(line.replace('mode:', '').rstrip())
                        if line.startswith('sequence_ids:'):
                            sequence_ids_str = line.replace('sequence_ids:', '').rstrip()
                            sequence_ids = [int(x) for x in sequence_ids_str.split(',')]
        return enabled, mode, sequence_ids

    def write_sequence_autorun(self, enabled, mode, sequence_ids):
        with self.lock:
            with open(self.sequence_autorun_status_file, 'w') as f:
                f.write("# THIS IS A GENERATED FILE\n")
                f.write('enabled:' + str(enabled).lower() + "\n")
                f.write('mode:' + str(mode) + "\n")
                f.write('sequence_ids:' + str(','.join(str(x) for x in sequence_ids)) + "\n")
       
        self.sequence_execution_index = -1
        self.enabled, self.mode, self.sequence_ids = self.read_sequence_autorun_status()
        return (self.enabled == enabled) and (self.mode == mode) and (self.sequence_ids == sequence_ids)

    def send_calibration_command(self):
        try:
            rospy.wait_for_service('/niryo_one/calibrate_motors', 0.1)
            start_calibration = rospy.ServiceProxy('/niryo_one/calibrate_motors', SetInt)
            start_calibration(1) # 1 : calibration auto
        except (rospy.ServiceException, rospy.ROSException), e:
            return False
        rospy.sleep(1)
        return True

    def activate_learning_mode(self, activate):
        try: 
            rospy.wait_for_service('/niryo_one/activate_learning_mode', 0.1)
            activate_learning_mode = rospy.ServiceProxy('/niryo_one/activate_learning_mode', SetInt)
            activate_learning_mode(int(activate))
        except (rospy.ServiceException, rospy.ROSException), e:
            return False
        rospy.sleep(1)
        return True

    def execute_sequence(self, seq_id):
        goal = SequenceGoal()
        goal.cmd_type = SequenceActionType.EXECUTE_FROM_ID
        goal.sequence_id = seq_id
        goal.sequence = Sequence()
        self.sequence_action_client.send_goal(goal)
        self.sequence_action_client.wait_for_result()
    
    # todo : after service with saved positions is developed, call the service
    # and remove hard-coded values here
    def send_robot_to_sleep_position(self):
        client = actionlib.SimpleActionClient('/niryo_one/commander/robot_action', RobotMoveAction)
        if not client.wait_for_server(rospy.Duration(1.0)):
            return
        goal = RobotMoveGoal()
        goal.cmd.cmd_type = MoveCommandType.JOINTS
        goal.cmd.joints = [0,0,-1.4,0,0,0]
        client.send_goal(goal)
        client.wait_for_result()

    # This thread is always alive when the node is alive
    # When activated, will launch each previously stored sequence
    # Before launching a sequence, it will make sure that calibration 
    # is done, and learning mode is deactivated
    def execute_sequence_autorun_thread(self):
        self.sequence_execution_index = -1
        while True:
            rospy.sleep(0.1)
            if self.activated:
                # 1. Check calibration
                if (self.calibration_needed == None) or \
                    (self.calibration_in_progress == None) or \
                    (self.learning_mode_on == None):
                    continue # haven't received hardware status yet
                if self.calibration_in_progress:
                    continue # wait until calibration is finished
                # 2. Execute calibration if needed
                if self.calibration_needed:
                    self.send_calibration_command()
                    continue
                # 3. Check learning mode - deactivate if needed
                if self.learning_mode_on:
                    self.activate_learning_mode(False)
                    continue
                # 4. Increment sequence index
                if len(self.sequence_ids) == 0:
                    continue # No sequence to launch
                self.sequence_execution_index += 1
                if self.sequence_execution_index >= len(self.sequence_ids):
                    self.sequence_execution_index = 0
                # 5. Launch sequence
                self.execute_sequence(self.sequence_ids[self.sequence_execution_index])
                # 6. Execute specific mode action
                if self.mode == SequenceAutorunMode.LOOP: # nothing to do, keep going with next sequence
                    pass
                elif self.mode == SequenceAutorunMode.ONE_SHOT: # stop after 1 sequence, wait for user input to continue
                    self.activated = False
                    self.flag_activate_learning_mode = True
            else:
                # if user deactivates sequence autorun, also go to sleep position
                # and activate learning mode after the sequence is finished
                if self.flag_activate_learning_mode:
                    self.send_robot_to_sleep_position()
                    if self.activate_learning_mode(True):
                        self.flag_activate_learning_mode = False

    def __init__(self):
        self.sequence_autorun_status_file = rospy.get_param("~sequence_autorun_status_file")
        self.sequence_autorun_status_file = os.path.expanduser(self.sequence_autorun_status_file)
        self.lock = Lock()
        self.enabled, self.mode, self.sequence_ids = self.read_sequence_autorun_status()
        self.activated = False
        self.sequence_execution_index = -1
        self.flag_activate_learning_mode = False

        self.calibration_needed = None
        self.calibration_in_progress = None
        self.hardware_status_subscriber = rospy.Subscriber(
                '/niryo_one/hardware_status', HardwareStatus, self.sub_hardware_status)

        self.learning_mode_on = None
        self.learning_mode_subscriber = rospy.Subscriber(
                '/niryo_one/learning_mode', Bool, self.sub_learning_mode)

        # Wait for sequence action server to finish setup
        self.sequence_action_client = actionlib.SimpleActionClient('niryo_one/sequences/execute', SequenceAction)
        self.sequence_action_client.wait_for_server()
        
        self.sequence_autorun_status_publisher = rospy.Publisher(
                '/niryo_one/sequences/sequence_autorun_status', SequenceAutorunStatus, queue_size=10)
        self.timer = rospy.Timer(rospy.Duration(3.0), self.publish_sequence_autorun_status)
        
        self.set_sequence_autorun_server = rospy.Service(
                '/niryo_one/sequences/set_sequence_autorun', SetSequenceAutorun, self.callback_set_sequence_autorun)

        self.trigger_sequence_autorun = rospy.Service(
                '/niryo_one/sequences/trigger_sequence_autorun', SetInt, self.callback_trigger_sequence_autorun)

        self.sequence_autorun_thread = Thread(name="worker", target=self.execute_sequence_autorun_thread)
        self.sequence_autorun_thread.setDaemon(True)
        self.sequence_autorun_thread.start()

        rospy.loginfo('Init Sequence autorun OK')

    def sub_hardware_status(self, msg):
        self.calibration_needed = msg.calibration_needed
        self.calibration_in_progress = msg.calibration_in_progress
    
    def sub_learning_mode(self, msg):
        self.learning_mode_on = msg.data

    def create_response(self, status, message):
        return {'status': status, 'message': message}
    
    def callback_set_sequence_autorun(self, req):
        if self.activated:
            return self.create_response(400, 'Cannot set Sequence Autorun while activated')
        if not self.write_sequence_autorun(req.enable, req.mode, list(req.sequence_ids)):
            return self.create_response(400, 'Failed to write sequence autorun into file')
        self.publish_sequence_autorun_status(None)
        return self.create_response(200, 'Sequence Autorun has been set')

    def callback_trigger_sequence_autorun(self, req):
        if not self.enabled:
            return self.create_response(400, 'Sequence Autorun is not enabled')
        self.activated = not self.activated 
        if self.activated:
            return self.create_response(200, 'Sequence Autorun has been activated')
        else:
            self.flag_activate_learning_mode = True
            return self.create_response(200, 'Sequence Autorun has been deactivated')
        
    def publish_sequence_autorun_status(self, event):
        msg = SequenceAutorunStatus()
        msg.enabled = self.enabled
        msg.mode = self.mode
        msg.sequence_ids = self.sequence_ids
        self.sequence_autorun_status_publisher.publish(msg)

if __name__ == '__main__':
    #rospy.init_node('sequence_autorun_mode_test')
    #s = SequenceAutorun()
    #rospy.spin()
    pass
