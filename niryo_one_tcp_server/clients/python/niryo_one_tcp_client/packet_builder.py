#!/usr/bin/env python

# packet_builder.py
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

from .enums import *


class PacketBuilder:
    class NotEnoughParameterException(Exception):
        pass

    class InvalidValueException(Exception):
        pass

    class UnknownCommandException(Exception):
        def __init__(self, command_enum):
            super(Exception, self).__init__("Unknown command given: {}".format(command_enum.name))
        pass

    class __CommandElement:
        def __init__(self, command_string, packet_builder_function):
            self.string_representation = command_string
            self.packet_builder_function = packet_builder_function

    def __init__(self):
        self.__command_elements_dict = {
            Command.CALIBRATE: self.__CommandElement("CALIBRATE", self.__calibrate),
            Command.SET_LEARNING_MODE: self.__CommandElement("SET_LEARNING_MODE", self.__set_learning_mode),
            Command.MOVE_JOINTS: self.__CommandElement("MOVE_JOINTS", self.__move_joints),
            Command.MOVE_POSE: self.__CommandElement("MOVE_POSE", self.__move_pose),
            Command.SHIFT_POSE: self.__CommandElement("SHIFT_POSE", self.__shift_pose),
            Command.SET_ARM_MAX_VELOCITY: self.__CommandElement("SET_ARM_MAX_VELOCITY", self.__set_arm_max_velocity),
            Command.ENABLE_JOYSTICK: self.__CommandElement("ENABLE_JOYSTICK", self.__enable_joystick),
            Command.SET_PIN_MODE: self.__CommandElement("SET_PIN_MODE", self.__set_pin_mode),
            Command.DIGITAL_WRITE: self.__CommandElement("DIGITAL_WRITE", self.__digital_write),
            Command.DIGITAL_READ: self.__CommandElement("DIGITAL_READ", self.__digital_read),
            Command.CHANGE_TOOL: self.__CommandElement("CHANGE_TOOL", self.__change_tool),
            Command.OPEN_GRIPPER: self.__CommandElement("OPEN_GRIPPER", self.__open_gripper),
            Command.CLOSE_GRIPPER: self.__CommandElement("CLOSE_GRIPPER", self.__close_gripper),
            Command.PULL_AIR_VACUUM_PUMP: self.__CommandElement("PULL_AIR_VACUUM_PUMP", self.__pull_air_vacuum_pump),
            Command.PUSH_AIR_VACUUM_PUMP: self.__CommandElement("PUSH_AIR_VACUUM_PUMP", self.__push_air_vacuum_pump),
            Command.SETUP_ELECTROMAGNET: self.__CommandElement("SETUP_ELECTROMAGNET", self.__setup_electromagnet),
            Command.ACTIVATE_ELECTROMAGNET: self.__CommandElement("ACTIVATE_ELECTROMAGNET",self.__activate_electromagnet),
            Command.DEACTIVATE_ELECTROMAGNET: self.__CommandElement("DEACTIVATE_ELECTROMAGNET", self.__deactivate_electromagnet),
            Command.GET_JOINTS: self.__CommandElement("GET_JOINTS", self.__get_joints),
            Command.GET_POSE: self.__CommandElement("GET_POSE", self.__get_pose),
            Command.GET_HARDWARE_STATUS: self.__CommandElement("GET_HARDWARE_STATUS", self.__get_hardware_status),
            Command.GET_LEARNING_MODE: self.__CommandElement("GET_LEARNING_MODE", self.__get_learning_mode),
            Command.GET_DIGITAL_IO_STATE: self.__CommandElement("GET_DIGITAL_IO_STATE", self.__get_digital_io_state),
        }

    def __build_packet_with_parameter(self, command_type, parameter_list):
        packet = self.__command_elements_dict[command_type].string_representation + ":"
        counter_param = 0
        for parameter in parameter_list:
            if isinstance(parameter, Enum):
                packet += parameter.name
            elif isinstance(parameter, bool):
                packet += str(parameter).upper()
            else:
                packet += str(parameter)
            counter_param += 1
            if counter_param < len(parameter_list):
                packet += ","
        return packet

    def __build_packet_without_parameter(self, command_type):
        return self.__command_elements_dict[command_type].string_representation

    def __calibrate(self, parameter_list):
        if parameter_list is None or len(parameter_list) < 1:
            raise self.NotEnoughParameterException(
                "One parameter expected [AUTO / MANUAL], {} parameters given".format(len(parameter_list)))
        if not isinstance(parameter_list[0], CalibrateMode):
            raise self.InvalidValueException(
                " Expected CalibrateMode enum parameter, given: {}".format(type(parameter_list[0])))

        return self.__build_packet_with_parameter(Command.CALIBRATE, parameter_list)

    def __set_learning_mode(self, parameter_list):
        if parameter_list is None or len(parameter_list) < 1:
            raise self.NotEnoughParameterException(
                "One parameter expected [True / False], {} parameters given".format(len(parameter_list)))
        if not isinstance(parameter_list[0], bool):
            raise self.InvalidValueException(" Expected bool parameter, given: {}".format(type(parameter_list[0])))

        return self.__build_packet_with_parameter(Command.SET_LEARNING_MODE, parameter_list)

    def __move_joints(self, parameter_list):
        if parameter_list is None or len(parameter_list) < 6:
            raise self.NotEnoughParameterException(
                "Six parameters expected [j1, j2, j3, j4, j5, j6], {} parameters given".format(len(parameter_list)))
        for parameter in parameter_list:
            if not isinstance(parameter, float):
                raise self.InvalidValueException(" Expected float parameters, given: {}".format(type(parameter)))
        return self.__build_packet_with_parameter(Command.MOVE_JOINTS, parameter_list)

    def __move_pose(self, parameter_list):
        if parameter_list is None or len(parameter_list) < 6:
            raise self.NotEnoughParameterException(
                "Six parameters expected [x, y, z, roll, pitch, yaw], {} parameters given".format(len(parameter_list)))
        for parameter in parameter_list:
            if not isinstance(parameter, float):
                raise self.InvalidValueException(" Expected float parameters, given: {}".format(type(parameter)))
        return self.__build_packet_with_parameter(Command.MOVE_POSE, parameter_list)

    def __shift_pose(self, parameter_list):
        if parameter_list is None or len(parameter_list) < 2:
            raise self.NotEnoughParameterException(
                "Two parameters expected [axis, shift_value], {} parameters given".format(len(parameter_list)))
        axis = parameter_list[0]
        if not isinstance(axis, RobotAxis):
            raise self.InvalidValueException(" Expected RobotAxis enum, given: {}".format(type(axis)))

        shift_value = parameter_list[1]
        if not isinstance(shift_value, float):
            raise self.InvalidValueException(" Expected float parameter, given: {}".format(type(shift_value)))

        return self.__build_packet_with_parameter(Command.SHIFT_POSE, parameter_list)

    def __set_arm_max_velocity(self, parameter_list):
        if parameter_list is None or len(parameter_list) < 1:
            raise self.NotEnoughParameterException(
                "One parameter expected [percentage], {} parameters given".format(len(parameter_list)))

        percentage = parameter_list[0]
        if not isinstance(percentage, int) or percentage not in range(0, 101):
            raise self.InvalidValueException(" Expected a percentage, given: {}".format(percentage))
        return self.__build_packet_with_parameter(Command.SET_ARM_MAX_VELOCITY, parameter_list)

    def __enable_joystick(self, parameter_list):
        if parameter_list is None or len(parameter_list) < 1:
            raise self.NotEnoughParameterException(
                "One parameter expected [True / False], {} parameters given".format(len(parameter_list)))

        if not isinstance(parameter_list[0], bool):
            raise self.InvalidValueException(" Expected bool parameter, given: {}".format(type(parameter_list[0])))
        return self.__build_packet_with_parameter(Command.ENABLE_JOYSTICK, parameter_list)

    def __set_pin_mode(self, parameter_list):
        if parameter_list is None or len(parameter_list) < 2:
            raise self.NotEnoughParameterException(
                "Two parameters expected [pin, pin_mode], {} parameters given".format(len(parameter_list)))

        if not isinstance(parameter_list[0], RobotPin):
            raise self.InvalidValueException(" Expected RobotPin enum parameter, given: {}".format(type(parameter_list[0])))

        if not isinstance(parameter_list[1], PinMode):
            raise self.InvalidValueException(" Expected PinMode enum parameter, given: {}".format(type(parameter_list[1])))
        return self.__build_packet_with_parameter(Command.SET_PIN_MODE, parameter_list)

    def __digital_write(self, parameter_list):
        if parameter_list is None or len(parameter_list) < 2:
            raise self.NotEnoughParameterException(
                "Two parameters expected [pin, pin_state], {} parameters given".format(len(parameter_list)))

        pin = parameter_list[0]
        if not isinstance(pin, RobotPin):
            raise self.InvalidValueException(" Expected RobotPin enum, given: {}".format(type(pin)))

        state = parameter_list[1]
        if not isinstance(state, DigitalState):
            raise self.InvalidValueException(" Expected DigitalState enum parameter, given: {}".format(type(state)))
        return self.__build_packet_with_parameter(Command.DIGITAL_WRITE, parameter_list)

    def __digital_read(self, parameter_list):
        if parameter_list is None or len(parameter_list) < 1:
            raise self.NotEnoughParameterException(
                "One parameter expected [pin], {} parameters given".format(len(parameter_list)))

        pin = parameter_list[0]
        if not isinstance(pin, RobotPin):
            raise self.InvalidValueException(" Expected RobotPin enum, given: {}".format(type(pin)))
        return self.__build_packet_with_parameter(Command.DIGITAL_READ, parameter_list)

    def __change_tool(self, parameter_list):
        if parameter_list is None or len(parameter_list) < 1:
            raise self.NotEnoughParameterException(
                "Two parameters expected [tool], {} parameters given".format(len(parameter_list)))

        tool = parameter_list[0]
        if not isinstance(tool, RobotTool):
            raise self.InvalidValueException(" Expected RobotTool enum, given: {}".format(type(tool)))
        return self.__build_packet_with_parameter(Command.CHANGE_TOOL, parameter_list)

    def __open_gripper(self, parameter_list):
        if parameter_list is None or len(parameter_list) < 2:
            raise self.NotEnoughParameterException(
                "Two parameters expected [gripper_type, speed], {} parameters given".format(len(parameter_list)))

        gripper_type = parameter_list[0]
        if not isinstance(gripper_type, RobotTool):
            raise self.InvalidValueException(" Expected RobotTool enum, given: {}".format(type(gripper_type)))

        speed = parameter_list[1]
        if not isinstance(speed, int):
            raise self.InvalidValueException(" Expected an integer, given: {}".format(type(speed)))
        return self.__build_packet_with_parameter(Command.OPEN_GRIPPER, parameter_list)

    def __close_gripper(self, parameter_list):
        if parameter_list is None or len(parameter_list) < 2:
            raise self.NotEnoughParameterException(
                "Two parameters expected [gripper_type, speed], {} parameters given".format(len(parameter_list)))

        gripper_type = parameter_list[0]
        if not isinstance(gripper_type, RobotTool):
            raise self.InvalidValueException(" Expected RobotTool enum, given: {}".format(type(gripper_type)))

        speed = parameter_list[1]
        if not isinstance(speed, int):
            raise self.InvalidValueException(" Expected an integer, given: {}".format(type(speed)))
        return self.__build_packet_with_parameter(Command.CLOSE_GRIPPER, parameter_list)

    def __pull_air_vacuum_pump(self, parameter_list):
        if parameter_list is None or len(parameter_list) < 1:
            raise self.NotEnoughParameterException(
                "Two parameters expected [vacuum_pump_type], {} parameters given".format(len(parameter_list)))

        vacuum_pump_type = parameter_list[0]
        if not isinstance(vacuum_pump_type, RobotTool):
            raise self.InvalidValueException(" Expected RobotTool enum, given: {}".format(type(vacuum_pump_type)))

        return self.__build_packet_with_parameter(Command.PULL_AIR_VACUUM_PUMP, parameter_list)

    def __push_air_vacuum_pump(self, parameter_list):
        if parameter_list is None or len(parameter_list) < 1:
            raise self.NotEnoughParameterException(
                "Two parameters expected [vacuum_pump_type], {} parameters given".format(len(parameter_list)))

        vacuum_pump_type = parameter_list[0]
        if not isinstance(vacuum_pump_type, RobotTool):
            raise self.InvalidValueException(" Expected RobotTool enum, given: {}".format(type(vacuum_pump_type)))

        return self.__build_packet_with_parameter(Command.PUSH_AIR_VACUUM_PUMP, parameter_list)

    def __setup_electromagnet(self, parameter_list):
        if parameter_list is None or len(parameter_list) < 2:
            raise self.NotEnoughParameterException(
                "Two parameters expected [electromagnet_type, pin], {} parameters given".format(len(parameter_list)))

        electromagnet_type = parameter_list[0]
        if not isinstance(electromagnet_type, RobotTool):
            raise self.InvalidValueException(" Expected RobotTool enum, given: {}".format(type(electromagnet_type)))

        pin = parameter_list[1]
        if not isinstance(pin, RobotPin):
            raise self.InvalidValueException(" Expected RobotPin enum, given: {}".format(type(pin)))

        return self.__build_packet_with_parameter(Command.SETUP_ELECTROMAGNET, parameter_list)

    def __activate_electromagnet(self, parameter_list):
        if parameter_list is None or len(parameter_list) < 2:
            raise self.NotEnoughParameterException(
                "Two parameters expected [electromagnet_type, pin], {} parameters given".format(len(parameter_list)))

        electromagnet_type = parameter_list[0]
        if not isinstance(electromagnet_type, RobotTool):
            raise self.InvalidValueException(" Expected RobotTool enum, given: {}".format(type(electromagnet_type)))

        pin = parameter_list[1]
        if not isinstance(pin, RobotPin):
            raise self.InvalidValueException(" Expected RobotPin enum, given: {}".format(type(pin)))

        return self.__build_packet_with_parameter(Command.ACTIVATE_ELECTROMAGNET, parameter_list)

    def __deactivate_electromagnet(self, parameter_list):
        if parameter_list is None or len(parameter_list) < 2:
            raise self.NotEnoughParameterException(
                "Two parameters expected [electromagnet_type, pin], {} parameters given".format(len(parameter_list)))

        electromagnet_type = parameter_list[0]
        if not isinstance(electromagnet_type, RobotTool):
            raise self.InvalidValueException(" Expected RobotTool enum, given: {}".format(type(electromagnet_type)))

        pin = parameter_list[1]
        if not isinstance(pin, RobotPin):
            raise self.InvalidValueException(" Expected RobotPin enum, given: {}".format(type(pin)))

        return self.__build_packet_with_parameter(Command.DEACTIVATE_ELECTROMAGNET, parameter_list)

    def __get_joints(self):
        return self.__build_packet_without_parameter(Command.GET_JOINTS)

    def __get_pose(self):
        return self.__build_packet_without_parameter(Command.GET_POSE)

    def __get_hardware_status(self):
        return self.__build_packet_without_parameter(Command.GET_HARDWARE_STATUS)

    def __get_learning_mode(self):
        return self.__build_packet_without_parameter(Command.GET_LEARNING_MODE)

    def __get_digital_io_state(self):
        return self.__build_packet_without_parameter(Command.GET_DIGITAL_IO_STATE)

    def build_command_packet(self, command_type, parameters=None):
        if command_type in self.__command_elements_dict:
            try:
                if parameters is None:
                    return self.__command_elements_dict[command_type].packet_builder_function()
                else:
                    return self.__command_elements_dict[command_type].packet_builder_function(parameters)
            except self.InvalidValueException as e:
                raise self.InvalidValueException("[{}] ".format(command_type) + str(e))
            except self.NotEnoughParameterException as e:
                raise self.NotEnoughParameterException("[{}] ".format(command_type) + str(e))
        raise self.UnknownCommandException(command_type)
