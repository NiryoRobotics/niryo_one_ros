#!/usr/bin/env python
# position.py
# Copyright (C) 2018 Niryo
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



class Position:
    class RPY: 
       
        def __init__(self, roll = 0, pitch = 0, yaw = 0): 
            self.roll = roll
            self.pitch = pitch 
            self.yaw = yaw
    class Point: 

        def __init__(self, x = 0, y = 0, z = 0 ): 
            self.x = x
            self.y = y
            self.z = z

    class Quaternion: 

        def __init__(self, x = 0, y = 0, z = 0, w = 0): 
            self.x = x
            self.y = y 
            self.z = z
            self.w = w

    def __init__(self, name = "", joints = [0,0,0,0,0,0], rpy = RPY(), point = Point(), quaternion = Quaternion()): 

        self.name = name 
        self.joints = joints 
        self.rpy = rpy
        self.point = point
        self.quaternion = quaternion

       
