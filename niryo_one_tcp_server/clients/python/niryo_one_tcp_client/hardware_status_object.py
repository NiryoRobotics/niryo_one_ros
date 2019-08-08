#!/usr/bin/env python

# hardware_status_object.py
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


class HardwareStatusObject:
    def __init__(self, rpi_temperature, hardware_version, connection_up,
                 error_message, calibration_needed, calibration_in_progress,
                 motor_names, motor_types,
                 temperatures, voltages, hardware_errors):
        self.rpi_temperature = rpi_temperature
        self.hardware_version = hardware_version
        self.connection_up = connection_up
        self.error_message = error_message
        self.calibration_needed = calibration_needed
        self.calibration_in_progress = calibration_in_progress
        self.motor_names = motor_names
        self.motor_types = motor_types
        self.temperatures = temperatures
        self.voltages = voltages
        self.hardware_errors = hardware_errors
