#!/usr/bin/env python

# tcp_client.py
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

import socket

from .pose_object import PoseObject
from .hardware_status_object import HardwareStatusObject
from .packet_builder import PacketBuilder
from .enums import Command
from .digital_pin_object import DigitalPinObject
import re
import ast


class NiryoOneClient:
    class HostNotReachableException(Exception):
        def __init__(self):
            super(Exception, self).__init__("Unable to communicate with robot server, please verify your network.")

    class ClientNotConnectedException(Exception):
        def __init__(self):
            super(Exception, self).__init__("You're not connected to  the robot.")

    class InvalidAnswerException(Exception):
        def __init__(self, answer):
            super(Exception, self).__init__(
                "An invalid answer has been received. Format expected: COMMAND:[OK[, data_answer]] / [KO, reason].\n"
                + "A problem occurred with: '" + answer + "'")

    def __init__(self, timeout=5):
        self.__port = 40001
        self.__is_running = True
        self.__is_connected = False
        self.__timeout = timeout
        self.__client_socket = None
        self.__packet_builder = PacketBuilder()

    def __del__(self):
        self.quit()

    def quit(self):
        self.__is_running = False
        self.__shutdown_connection()
        self.__client_socket = None

    def __shutdown_connection(self):
        if self.__client_socket is not None and self.__is_connected is True:
            try:
                self.__client_socket.shutdown(socket.SHUT_RDWR)
                self.__client_socket.close()
            except socket.error as e:
                pass
            self.__is_connected = False

    def connect(self, ip_address):
        self.__client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.__client_socket.settimeout(self.__timeout)
        try:
            self.__client_socket.connect((ip_address, self.__port))
        except socket.timeout:
            print("Unable to connect to the robot.")
            self.__shutdown_connection()
            self.__client_socket = None
        except socket.error as e:
            print("An error occurred while attempting to connect: {}".format(e))
            self.__shutdown_connection()
            self.__client_socket = None
        else:
            print("Connected to server ({}) on port: {}".format(ip_address, self.__port))
            self.__is_connected = True
            self.__client_socket.settimeout(None)

        return self.__is_connected

    def calibrate(self, calibrate_mode):
        self.send_command(Command.CALIBRATE, [calibrate_mode])
        return self.receive_answer()

    def set_learning_mode(self, enabled):
        self.send_command(Command.SET_LEARNING_MODE, [enabled])
        return self.receive_answer()

    def move_joints(self, j1, j2, j3, j4, j5, j6):
        self.send_command(Command.MOVE_JOINTS, [j1, j2, j3, j4, j5, j6])
        return self.receive_answer()

    def move_pose(self, x_pos, y_pos, z_pos, roll_rot, pitch_rot, yaw_rot):
        self.send_command(Command.MOVE_POSE, [x_pos, y_pos, z_pos, roll_rot, pitch_rot, yaw_rot])
        return self.receive_answer()

    def shift_pose(self, axis, shift_value):
        self.send_command(Command.SHIFT_POSE, [axis, shift_value])
        return self.receive_answer()

    def set_arm_max_velocity(self, percentage_speed):
        self.send_command(Command.SET_ARM_MAX_VELOCITY, [percentage_speed])
        return self.receive_answer()

    def enable_joystick(self, enabled):
        self.send_command(Command.ENABLE_JOYSTICK, [enabled])
        return self.receive_answer()

    def set_pin_mode(self, pin, pin_mode):
        self.send_command(Command.SET_PIN_MODE, [pin, pin_mode])
        return self.receive_answer()

    def digital_write(self, pin, digital_state):
        self.send_command(Command.DIGITAL_WRITE, [pin, digital_state])
        return self.receive_answer()

    def digital_read(self, pin):
        self.send_command(Command.DIGITAL_READ, [pin])
        status, data = self.receive_answer()
        if status is True:
            return status, int(data)
        return status, data

    def change_tool(self, tool):
        self.send_command(Command.CHANGE_TOOL, [tool])
        return self.receive_answer()

    def open_gripper(self, gripper, speed):
        self.send_command(Command.OPEN_GRIPPER, [gripper, speed])
        return self.receive_answer()

    def close_gripper(self, gripper, speed):
        self.send_command(Command.CLOSE_GRIPPER, [gripper, speed])
        return self.receive_answer()

    def pull_air_vacuum_pump(self, vacuum_pump):
        self.send_command(Command.PULL_AIR_VACUUM_PUMP, [vacuum_pump])
        return self.receive_answer()

    def push_air_vacuum_pump(self, vacuum_pump):
        self.send_command(Command.PUSH_AIR_VACUUM_PUMP, [vacuum_pump])
        return self.receive_answer()

    def setup_electromagnet(self, electromagnet, pin):
        self.send_command(Command.SETUP_ELECTROMAGNET, [electromagnet, pin])
        return self.receive_answer()

    def activate_electromagnet(self, electromagnet, pin):
        self.send_command(Command.ACTIVATE_ELECTROMAGNET, [electromagnet, pin])
        return self.receive_answer()

    def deactivate_electromagnet(self, electromagnet, pin):
        self.send_command(Command.DEACTIVATE_ELECTROMAGNET, [electromagnet, pin])
        return self.receive_answer()

    def get_saved_position_list(self):
        self.send_command(Command.GET_SAVED_POSITION_LIST)
        return self.receive_answer()

    def wait(self, duration):
        self.send_command(Command.WAIT, [duration])
        return self.receive_answer()

    def get_joints(self):
        self.send_command(Command.GET_JOINTS)
        status, data = self.receive_answer()
        if status is True:
            joint_array = map(float, data.split(','))
            return status, joint_array
        return status, data

    def get_pose(self):
        self.send_command(Command.GET_POSE)
        status, data = self.receive_answer()
        if status is True:
            pose_array = map(float, data.split(','))
            pose_object = PoseObject(*pose_array)
            return status, pose_object
        return status, data

    def get_hardware_status(self):
        self.send_command(Command.GET_HARDWARE_STATUS)
        status, data = self.receive_answer()
        if status is True:
            matches = re.findall('((?:\[[^\]]+\])|(?:\([^\)]+\))|True|False|\d+|\'\w*\')', data)
            if len(matches) != 11:
                print("[get_hardware_status] Incorrect answer received, cannot understand received format.")
                return status, data
            rpi_temperature = int(matches[0])
            hardware_version = int(matches[1])
            connection_up = bool(matches[2])
            error_message = matches[3].strip('\'')
            calibration_needed = int(matches[4])
            calibration_in_progress = bool(matches[5])

            motor_names = ast.literal_eval(matches[6])
            motor_types = ast.literal_eval(matches[7])

            temperatures = ast.literal_eval(matches[8])
            voltages = ast.literal_eval(matches[9])
            hardware_errors = ast.literal_eval(matches[10])

            hardware_status = HardwareStatusObject(rpi_temperature, hardware_version, connection_up, error_message,
                                                   calibration_needed, calibration_in_progress,
                                                   motor_names, motor_types,
                                                   temperatures, voltages, hardware_errors)
            return status, hardware_status
        return status, data

    def get_learning_mode(self):
        self.send_command(Command.GET_LEARNING_MODE)
        status, data = self.receive_answer()
        if status is True:
            return status, bool(data)
        return status, data

    def get_digital_io_state(self):
        self.send_command(Command.GET_DIGITAL_IO_STATE)
        status, data = self.receive_answer()
        if status is True:
            matches = re.findall('(\[\d+, ?\'\w+\', ?[0-1], \d+\])+', data)
            digital_pin_array = []
            for match in matches:
                elements = match.split(', ')
                pin_id = elements[0].lstrip('[')
                name = elements[1]
                mode = int(elements[2])
                state = int(elements[3].rstrip(']'))
                digital_pin_array.append(DigitalPinObject(pin_id, name, mode, state))
            return status, digital_pin_array
        return status, data

    def send_command(self, command_type, parameter_list=None):
        if self.__is_connected is False:
            raise self.ClientNotConnectedException()
        send_success = False
        if self.__client_socket is not None:
            try:
                packet = self.__packet_builder.build_command_packet(command_type, parameter_list)
                self.__client_socket.send(packet)
            except socket.error as e:
                print(e)
                raise self.HostNotReachableException()
        return send_success

    def receive_answer(self):
        READ_SIZE = 512
        try:
            received = self.__client_socket.recv(READ_SIZE)
        except socket.error as e:
            print(e)
            raise self.HostNotReachableException()
        if not received:
            raise self.HostNotReachableException()
        received_split_list = received.split(':', 1)
        if len(received_split_list) != 2:
            raise self.InvalidAnswerException(received)
        command_answer = received_split_list[1]

        # If 'OK' with data or 'KO' with reason
        if ',' in command_answer:
            command_answer_split_list = command_answer.split(',', 1)
            if len(command_answer_split_list) != 2:
                raise self.InvalidAnswerException(command_answer)
            answer_status = command_answer_split_list[0]
            if answer_status == "OK" and answer_status == "KO":
                raise self.InvalidAnswerException(answer_status)
            answer_data = command_answer_split_list[1]
            return answer_status == "OK", answer_data

        if command_answer == "OK" and command_answer == "KO":
            raise self.InvalidAnswerException(command_answer)
        return command_answer == "OK", None
