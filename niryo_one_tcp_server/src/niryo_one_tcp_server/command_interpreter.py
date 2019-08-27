#!/usr/bin/env python

# command_interpreter.py
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

import rospy
import niryo_one_python_api.niryo_one_api as python_api


class TcpCommandException(Exception):
    pass


class CommandInterpreter:
    def __init__(self):
        self.__niryo_one = python_api.NiryoOne()
        self.__commands_dict = {
            "CALIBRATE": self.__calibrate,
            "SET_LEARNING_MODE": self.__set_learning_mode,
            "MOVE_JOINTS": self.__move_joints,
            "MOVE_POSE": self.__move_pose,
            "SHIFT_POSE": self.__shift_pose,
            "SET_ARM_MAX_VELOCITY": self.__set_arm_max_velocity,
            "ENABLE_JOYSTICK": self.__enable_joystick,
            "SET_PIN_MODE": self.__set_pin_mode,
            "DIGITAL_WRITE": self.__digital_write,
            "DIGITAL_READ": self.__digital_read,
            "CHANGE_TOOL": self.__change_tool,
            "OPEN_GRIPPER": self.__open_gripper,
            "CLOSE_GRIPPER": self.__close_gripper,
            "PULL_AIR_VACUUM_PUMP": self.__pull_air_vacuum_pump,
            "PUSH_AIR_VACUUM_PUMP": self.__push_air_vacuum_pump,
            "SETUP_ELECTROMAGNET": self.__setup_electromagnet,
            "ACTIVATE_ELECTROMAGNET": self.__activate_electromagnet,
            "DEACTIVATE_ELECTROMAGNET": self.__deactivate_electromagnet,
            "GET_JOINTS": self.__get_joints,
            "GET_POSE": self.__get_pose,
            "GET_HARDWARE_STATUS": self.__get_hardware_status,
            "GET_LEARNING_MODE": self.__get_learning_mode,
            "GET_DIGITAL_IO_STATE": self.__get_digital_io_state
        }
        self.__axis_string_dict_convertor = {"X": python_api.AXIS_X,
                                             "Y": python_api.AXIS_Y,
                                             "Z": python_api.AXIS_Z,
                                             "ROLL": python_api.ROT_ROLL,
                                             "PITCH": python_api.ROT_PITCH,
                                             "YAW": python_api.ROT_YAW
                                             }
        self.__pin_nbr_string_dict_convertor = {"GPIO_1A": python_api.GPIO_1A,
                                                "GPIO_1B": python_api.GPIO_1B,
                                                "GPIO_1C": python_api.GPIO_1C,
                                                "GPIO_2A": python_api.GPIO_2A,
                                                "GPIO_2B": python_api.GPIO_2B,
                                                "GPIO_2C": python_api.GPIO_2C
                                                }
        self.__pin_mode_string_dict_convertor = {"OUTPUT": python_api.PIN_MODE_OUTPUT,
                                                 "INPUT": python_api.PIN_MODE_INPUT
                                                 }
        self.__digital_state_string_dict_convertor = {"HIGH": python_api.PIN_HIGH,
                                                      "LOW": python_api.PIN_LOW
                                                      }
        self.__boolean_string_dict_convertor = {"TRUE": True,
                                                "FALSE": False
                                                }

        self.__available_tools_string_dict_convertor = {"NONE": python_api.TOOL_NONE,
                                                        "GRIPPER_1": python_api.TOOL_GRIPPER_1_ID,
                                                        "GRIPPER_2": python_api.TOOL_GRIPPER_2_ID,
                                                        "GRIPPER_3": python_api.TOOL_GRIPPER_3_ID,
                                                        "ELECTROMAGNET_1": python_api.TOOL_ELECTROMAGNET_1_ID,
                                                        "VACUUM_PUMP_1": python_api.TOOL_VACUUM_PUMP_1_ID,
                                                        }
        self.__available_gripper_tools_string_dict_convertor = {"NONE": python_api.TOOL_NONE,
                                                                "GRIPPER_1": python_api.TOOL_GRIPPER_1_ID,
                                                                "GRIPPER_2": python_api.TOOL_GRIPPER_2_ID,
                                                                "GRIPPER_3": python_api.TOOL_GRIPPER_3_ID
                                                                }

    def __raise_exception_expected_choice(self, expected_choice, given):
        raise TcpCommandException("Expected one of the following: " + expected_choice + ".\nGiven: " + given)

    def __raise_exception_expected_type(self, expected_type, given):
        raise TcpCommandException("Expected the following type: " + expected_type + ".\nGiven: " + given)

    def __raise_exception_expected_parameters_nbr(self, expected_nbr, given):
        raise TcpCommandException(str(expected_nbr) + " parameters expected, given: " + str(given))

    def interpret_command(self, command_received):
        if not isinstance(command_received, basestring):
            raise ValueError("Cannot interpret command of incorrect type: " + type(command_received))
        split_list = command_received.split(":")
        if len(split_list) != 2:
            rospy.loginfo("Incorrect command format: ", command_received)
            return command_received + ":KO,Incorrect command format."
        command_string_part = split_list[0].rstrip('\r\n')
        ret_string = command_string_part + ":"
        if command_string_part in self.__commands_dict:
            try:
                # Functions with parameter
                if len(split_list) is 2:
                    parameters_string_part = split_list[1]
                    ret = self.__commands_dict[command_string_part](parameters_string_part)
                # Getter functions (functions without parameter)
                else:
                    ret = self.__commands_dict[command_string_part]()
                return ret_string + ret
            except TypeError as e:
                rospy.loginfo("Incorrect number of parameter(s) given. " + str(e))
                return ret_string + "KO,\"" + "Incorrect number of parameter(s) given.\""
            except python_api.NiryoOneException as e:
                rospy.loginfo(e)
                return ret_string + "KO,\"" + str(e) + "\""
            except TcpCommandException as e:
                rospy.loginfo("Incorrect parameter(s) given to : " + command_string_part + " function. " + str(e))
                return ret_string + "KO,\"" + str(e) + "\""
        else:
            return command_string_part + ": " + "KO,\"" + "Unknown command\""

    def __successfull_answer(self, param_list=None):
        string_answer = "OK"
        if param_list is not None:
            string_answer += ","
            counter_param = 0
            for param in param_list:
                string_answer += str(param)
                counter_param += 1
                if counter_param < len(param_list):
                    string_answer += ","
        return string_answer

    def __calibrate(self, param_string):
        calibrate_mode_string = param_string.rstrip('\r\n')
        if calibrate_mode_string != "MANUAL" and calibrate_mode_string != "AUTO":
            self.__raise_exception_expected_choice("[MANUAL, AUTO]", param_string)

        if calibrate_mode_string == "MANUAL":
            self.__niryo_one.calibrate_manual()
        else:
            self.__niryo_one.calibrate_auto()
        return self.__successfull_answer()

    def __set_learning_mode(self, param_string):
        state_string = param_string.rstrip('\r\n')
        if state_string not in self.__boolean_string_dict_convertor:
            self.__raise_exception_expected_choice("[TRUE, FALSE]", param_string)
        state = self.__boolean_string_dict_convertor[state_string]
        self.__niryo_one.activate_learning_mode(state)
        return self.__successfull_answer()

    def __move_joints(self, param_string):
        parameters_string_array = param_string.split(",")
        nbr_parameters = len(parameters_string_array)

        if nbr_parameters != 6:
            self.__raise_exception_expected_parameters_nbr(6, nbr_parameters)

        try:
            joints_value_array = map(float, parameters_string_array)
        except ValueError as e:
            rospy.loginfo(str(e))
            self.__raise_exception_expected_type("float", parameters_string_array)
        else:
            self.__niryo_one.move_joints(joints_value_array)
        return self.__successfull_answer()

    def __move_pose(self, param_string):
        parameters_string_array = param_string.split(',')
        nbr_parameters = len(parameters_string_array)

        if nbr_parameters != 6:
            self.__raise_exception_expected_parameters_nbr(6, nbr_parameters)
        try:
            parameters_value_array = map(float, parameters_string_array)
        except ValueError as e:
            rospy.loginfo(str(e))
            self.__raise_exception_expected_type("float", parameters_string_array)
        else:
            self.__niryo_one.move_pose(*parameters_value_array)
        return self.__successfull_answer()

    def __shift_pose(self, param_string):
        parameters_string_array = param_string.split(',')
        nbr_parameters = len(parameters_string_array)

        if nbr_parameters != 2:
            self.__raise_exception_expected_parameters_nbr(2, nbr_parameters)

        axis_string = parameters_string_array[0]
        if axis_string not in self.__axis_string_dict_convertor:
            self.__raise_exception_expected_choice("[X, Y, Z, ROLL, PITCH, YAW]", axis_string)
        axis = self.__axis_string_dict_convertor[axis_string]

        try:
            value = float(parameters_string_array[1])
        except ValueError as e:
            rospy.loginfo(str(e))
            self.__raise_exception_expected_type("float", parameters_string_array[1])
        else:
            self.__niryo_one.shift_pose(axis, value)
        return self.__successfull_answer()

    def __set_arm_max_velocity(self, param_string):
        try:
            value = int(param_string)
        except ValueError as e:
            rospy.loginfo(str(e))
            self.__raise_exception_expected_type("integer", param_string)
        else:
            if value < 0 or value > 100:
                self.__raise_exception_expected_choice("[0 => 100]", param_string)
            percentage = value
            self.__niryo_one.set_arm_max_velocity(percentage)
        return self.__successfull_answer()

    def __enable_joystick(self, param_string):
        state_string = param_string.rstrip('\r\n')
        if state_string not in self.__boolean_string_dict_convertor:
            self.__raise_exception_expected_choice("[TRUE, FALSE]", param_string)
        state = self.__boolean_string_dict_convertor[state_string]
        self.__niryo_one.enable_joystick(state)
        return self.__successfull_answer()

    def __set_pin_mode(self, param_string):
        parameters_string_array = param_string.split(",")
        nbr_parameters = len(parameters_string_array)

        if len(parameters_string_array) != 2:
            self.__raise_exception_expected_parameters_nbr(2, nbr_parameters)

        pin_string = parameters_string_array[0]
        if pin_string not in self.__pin_nbr_string_dict_convertor:
            self.__raise_exception_expected_choice("[GPIO_1A, GPIO_1B, GPIO_1C, GPIO_2A, GPIO_2B, GPIO_2C]",
                                                   parameters_string_array[0])
        pin = self.__pin_nbr_string_dict_convertor[pin_string]

        pin_mode_string = parameters_string_array[1].rstrip('\r\n')
        if pin_mode_string not in self.__pin_mode_string_dict_convertor:
            self.__raise_exception_expected_choice("[OUTPUT, INPUT]", parameters_string_array[1])
        mode = self.__pin_mode_string_dict_convertor[pin_mode_string]

        self.__niryo_one.pin_mode(pin, mode)
        return self.__successfull_answer()

    def __digital_write(self, param_string):
        parameters_string_array = param_string.split(",")
        nbr_parameters = len(parameters_string_array)

        if len(parameters_string_array) != 2:
            self.__raise_exception_expected_parameters_nbr(2, nbr_parameters)

        pin_string = parameters_string_array[0]
        if pin_string not in self.__pin_nbr_string_dict_convertor:
            self.__raise_exception_expected_choice("[GPIO_1A, GPIO_1B, GPIO_1C, GPIO_2A, GPIO_2B, GPIO_2C]",
                                                   parameters_string_array[0])
        pin = self.__pin_nbr_string_dict_convertor[pin_string]

        state_string = parameters_string_array[1].rstrip('\r\n')
        if state_string not in self.__digital_state_string_dict_convertor:
            self.__raise_exception_expected_choice("[HIGH, LOW]", parameters_string_array[1])
        state = self.__digital_state_string_dict_convertor[state_string]

        self.__niryo_one.digital_write(pin, state)
        return self.__successfull_answer()

    def __digital_read(self, param_string):
        parameters_string_array = param_string.split(",")
        nbr_parameters = len(parameters_string_array)

        if len(parameters_string_array) != 1:
            self.__raise_exception_expected_parameters_nbr(1, nbr_parameters)

        pin_string = parameters_string_array[0].rstrip('\r\n')
        if pin_string not in self.__pin_nbr_string_dict_convertor:
            self.__raise_exception_expected_choice("[GPIO_1A, GPIO_1B, GPIO_1C, GPIO_2A, GPIO_2B, GPIO_2C]",
                                                   parameters_string_array[0])
        pin = self.__pin_nbr_string_dict_convertor[pin_string]

        return self.__successfull_answer([str(self.__niryo_one.digital_read(pin))])

    def __change_tool(self, param_string):
        tool_string = param_string.rstrip('\r\n')
        if tool_string not in self.__available_tools_string_dict_convertor:
            self.__raise_exception_expected_choice(
                "[NONE, GRIPPER_1, GRIPPER_2, GRIPPER_3, ELECTROMAGNET_1, VACUUM_PUMP_1]", param_string)
        tool_id = self.__available_tools_string_dict_convertor[tool_string]

        self.__niryo_one.change_tool(tool_id)
        return self.__successfull_answer()

    def __open_gripper(self, param_string):
        parameters_string_array = param_string.split(",")
        nbr_parameters = len(parameters_string_array)

        if len(parameters_string_array) != 2:
            self.__raise_exception_expected_parameters_nbr(2, nbr_parameters)

        gripper_id_string = parameters_string_array[0]
        if gripper_id_string not in self.__available_gripper_tools_string_dict_convertor:
            self.__raise_exception_expected_choice("GRIPPER_1, GRIPPER_2, GRIPPER_3]", parameters_string_array[0])
        gripper_id = self.__available_tools_string_dict_convertor[gripper_id_string]

        speed_string = parameters_string_array[1]
        try:
            speed = int(speed_string)
        except ValueError as e:
            rospy.loginfo(str(e))
            self.__raise_exception_expected_type("integer", speed_string)
        else:
            self.__niryo_one.open_gripper(gripper_id, speed)
        return self.__successfull_answer()

    def __close_gripper(self, param_string):
        parameters_string_array = param_string.split(",")
        nbr_parameters = len(parameters_string_array)

        if len(parameters_string_array) != 2:
            self.__raise_exception_expected_parameters_nbr(2, nbr_parameters)

        gripper_id_string = parameters_string_array[0]
        if gripper_id_string not in self.__available_gripper_tools_string_dict_convertor:
            self.__raise_exception_expected_choice("[GRIPPER_1, GRIPPER_2, GRIPPER_3]", parameters_string_array[0])
        gripper_id = self.__available_tools_string_dict_convertor[gripper_id_string]

        speed_string = parameters_string_array[1]
        try:
            speed = int(speed_string)
        except ValueError as e:
            rospy.loginfo(str(e))
            self.__raise_exception_expected_type("integer", speed_string)
        else:
            self.__niryo_one.close_gripper(gripper_id, speed)
        return self.__successfull_answer()

    def __pull_air_vacuum_pump(self, param_string):
        vacuum_pump_id_string = param_string.rstrip('\r\n')
        if vacuum_pump_id_string not in self.__available_tools_string_dict_convertor:
            self.__raise_exception_expected_choice("[VACUUM_PUMP_1]", param_string)
        vacuum_pump_id = self.__available_tools_string_dict_convertor[vacuum_pump_id_string]

        self.__niryo_one.pull_air_vacuum_pump(vacuum_pump_id)
        return self.__successfull_answer()

    def __push_air_vacuum_pump(self, param_string):
        vacuum_pump_id_string = param_string.rstrip('\r\n')
        if vacuum_pump_id_string not in self.__available_tools_string_dict_convertor:
            self.__raise_exception_expected_choice("[VACUUM_PUMP_1]", param_string)
        vacuum_pump_id = self.__available_tools_string_dict_convertor[vacuum_pump_id_string]

        self.__niryo_one.push_air_vacuum_pump(vacuum_pump_id)
        return self.__successfull_answer()

    def __setup_electromagnet(self, param_string):
        parameters_string_array = param_string.split(",")
        nbr_parameters = len(parameters_string_array)

        if len(parameters_string_array) != 2:
            self.__raise_exception_expected_parameters_nbr(2, nbr_parameters)

        electromagnet_id_string = parameters_string_array[0]
        if electromagnet_id_string not in self.__available_tools_string_dict_convertor:
            self.__raise_exception_expected_choice("[electromagnet_1]", parameters_string_array[0])
        electromagnet_id = self.__available_tools_string_dict_convertor[electromagnet_id_string]

        pin_string = parameters_string_array[1].rstrip('\r\n')
        if pin_string not in self.__pin_nbr_string_dict_convertor:
            self.__raise_exception_expected_choice("[GPIO_1A, GPIO_1B, GPIO_1C, GPIO_2A, GPIO_2B, GPIO_2C]",
                                                   parameters_string_array[1])
        pin = self.__pin_nbr_string_dict_convertor[pin_string]

        self.__niryo_one.setup_electromagnet(electromagnet_id, pin)
        return self.__successfull_answer()

    def __activate_electromagnet(self, param_string):
        parameters_string_array = param_string.split(",")
        nbr_parameters = len(parameters_string_array)

        if len(parameters_string_array) != 2:
            self.__raise_exception_expected_parameters_nbr(2, nbr_parameters)

        electromagnet_id_string = parameters_string_array[0]
        if electromagnet_id_string not in self.__available_tools_string_dict_convertor:
            self.__raise_exception_expected_choice("[ELECTROMAGNET_1]", parameters_string_array[0])
        electromagnet_id = self.__available_tools_string_dict_convertor[electromagnet_id_string]

        pin_string = parameters_string_array[1].rstrip('\r\n')
        if pin_string not in self.__pin_nbr_string_dict_convertor:
            self.__raise_exception_expected_choice("[GPIO_1A, GPIO_1B, GPIO_1C, GPIO_2A, GPIO_2B, GPIO_2C]",
                                                   parameters_string_array[1])
        pin = self.__pin_nbr_string_dict_convertor[pin_string]

        self.__niryo_one.activate_electromagnet(electromagnet_id, pin)
        return self.__successfull_answer()

    def __deactivate_electromagnet(self, param_string):
        parameters_string_array = param_string.split(",")
        nbr_parameters = len(parameters_string_array)

        if len(parameters_string_array) != 2:
            self.__raise_exception_expected_parameters_nbr(2, nbr_parameters)

        electromagnet_id_string = parameters_string_array[0]
        if electromagnet_id_string not in self.__available_tools_string_dict_convertor:
            self.__raise_exception_expected_choice("[ELECTROMAGNET_1]", parameters_string_array[0])
        electromagnet_id = self.__available_tools_string_dict_convertor[electromagnet_id_string]

        pin_string = parameters_string_array[1].rstrip('\r\n')
        if pin_string not in self.__pin_nbr_string_dict_convertor:
            self.__raise_exception_expected_choice("[GPIO_1A, GPIO_1B, GPIO_1C, GPIO_2A, GPIO_2B, GPIO_2C]",
                                                   parameters_string_array[1])
        pin = self.__pin_nbr_string_dict_convertor[pin_string]

        self.__niryo_one.deactivate_electromagnet(electromagnet_id, pin)
        return self.__successfull_answer()

    def __get_joints(self):
        joints = self.__niryo_one.get_joints()
        return self.__successfull_answer(joints)

    def __get_pose(self):
        arm_pose = self.__niryo_one.get_arm_pose()
        data_answer = []
        data_answer.append(arm_pose.position.x)
        data_answer.append(arm_pose.position.y)
        data_answer.append(arm_pose.position.z)
        data_answer.append(arm_pose.rpy.roll)
        data_answer.append(arm_pose.rpy.pitch)
        data_answer.append(arm_pose.rpy.yaw)
        return self.__successfull_answer(data_answer)

    def __get_hardware_status(self):
        hw_status = self.__niryo_one.get_hardware_status()
        data_answer = []
        data_answer.append(hw_status.rpi_temperature)
        data_answer.append(hw_status.hardware_version)
        data_answer.append(hw_status.connection_up)
        data_answer.append("\'" + hw_status.error_message + "\'")
        data_answer.append(hw_status.calibration_needed)
        data_answer.append(hw_status.calibration_in_progress)
        data_answer.append(hw_status.motor_names)
        data_answer.append(hw_status.motor_types)
        data_answer.append(hw_status.temperatures)
        data_answer.append(hw_status.voltages)
        data_answer.append(hw_status.hardware_errors)
        return self.__successfull_answer(data_answer)

    def __get_learning_mode(self):
        is_learning_mode_enabled = self.__niryo_one.get_learning_mode()
        return self.__successfull_answer([is_learning_mode_enabled])

    def __get_digital_io_state(self):
        digital_io_state_array = self.__niryo_one.get_digital_io_state()
        data_answer = []
        for counter in range(0, len(digital_io_state_array.pins)):
            data_answer.append([digital_io_state_array.pins[counter],
                                digital_io_state_array.names[counter],
                                digital_io_state_array.modes[counter],
                                digital_io_state_array.states[counter]])
        return self.__successfull_answer(data_answer)
