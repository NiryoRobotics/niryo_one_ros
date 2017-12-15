#!/usr/bin/env python

# holding_register_data_block.py
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
from niryo_one_msgs.srv import SetInt

from niryo_one_modbus.niryo_one_data_block import NiryoOneDataBlock

"""
 - Each address contains a 16 bits value
 - READ/WRITE registers

 --> Used to give commands to the robot 
 ( ! the stored values correspond to the last given command,
 not the current robot state !)
"""

HR_LEARNING_MODE    = 300
HR_JOYSTICK_ENABLED = 301

class HoldingRegisterDataBlock(NiryoOneDataBlock):

    def __init__(self):
        super(HoldingRegisterDataBlock, self).__init__()
        
    # Override
    def setValues(self, address, values):
        self.process_command(address, values)
        super(HoldingRegisterDataBlock, self).setValues(address, values)

    def process_command(self, address, values):
        address -= 1
        if len(values) == 0:
            return

        if address == HR_LEARNING_MODE:
            self.activate_learning_mode(values[0])
        elif address == HR_JOYSTICK_ENABLED:
            self.enable_joystick(values[0])


    def activate_learning_mode(self, activate):
        activate = int(activate >= 1)
        self.call_ros_service('/niryo_one/activate_learning_mode', SetInt, [activate])

    def enable_joystick(self, enable):
        enable = int(enable >= 1)
        self.call_ros_service('/niryo_one/joystick_interface/enable', SetInt, [enable])

