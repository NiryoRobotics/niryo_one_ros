#!/usr/bin/env python

# pose_object.py
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


class PoseObject:
    def __init__(self, x, y, z, roll, pitch, yaw):
        # X (meter)
        self.x = x
        # Y (meter)
        self.y = y
        # Z (meter)
        self.z = z
        # Roll (radian)
        self.roll = roll
        # Pitch (radian)
        self.pitch = pitch
        # Yaw (radian)
        self.yaw = yaw

