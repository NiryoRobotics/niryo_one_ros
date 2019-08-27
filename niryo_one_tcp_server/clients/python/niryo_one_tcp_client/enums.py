#!/usr/bin/env python

# enums.py
# Copyright (C) 2019 Niryo
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

from enum import Enum, unique


@unique
class Command(Enum):
    CALIBRATE = 0
    SET_LEARNING_MODE = 1
    MOVE_JOINTS = 2
    MOVE_POSE = 3
    SHIFT_POSE = 4
    SET_ARM_MAX_VELOCITY = 5
    ENABLE_JOYSTICK = 6
    SET_PIN_MODE = 7
    DIGITAL_WRITE = 8
    DIGITAL_READ = 9
    CHANGE_TOOL = 10
    OPEN_GRIPPER = 11
    CLOSE_GRIPPER = 12
    PULL_AIR_VACUUM_PUMP = 13
    PUSH_AIR_VACUUM_PUMP = 14
    SETUP_ELECTROMAGNET = 15
    ACTIVATE_ELECTROMAGNET = 16
    DEACTIVATE_ELECTROMAGNET = 17
    GET_JOINTS = 18
    GET_POSE = 19
    GET_HARDWARE_STATUS = 20
    GET_LEARNING_MODE = 21
    GET_DIGITAL_IO_STATE = 22


@unique
class CalibrateMode(Enum):
    AUTO = 0
    MANUAL = 1


@unique
class PinMode(Enum):
    INPUT = 0
    OUTPUT = 1


@unique
class RobotPin(Enum):
    GPIO_1A = 0
    GPIO_1B = 1
    GPIO_1C = 2
    GPIO_2A = 3
    GPIO_2B = 4
    GPIO_2C = 5


@unique
class DigitalState(Enum):
    LOW = 0
    HIGH = 1


@unique
class RobotAxis(Enum):
    X = 0
    Y = 1
    Z = 2
    ROLL = 3
    PITCH = 4
    YAW = 5


@unique
class RobotTool(Enum):
    NONE = 0
    GRIPPER_1 = 1
    GRIPPER_2 = 2
    GRIPPER_3 = 3
    ELECTROMAGNET_1 = 20
    VACUUM_PUMP_1 = 30
