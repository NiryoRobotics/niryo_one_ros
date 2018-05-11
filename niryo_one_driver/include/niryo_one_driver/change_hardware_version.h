/*
    change_hardware_version.h
    Copyright (C) 2018 Niryo
    All rights reserved.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef CHANGE_HARDWARE_VERSION_H
#define CHANGE_HARDWARE_VERSION_H

#include <ros/ros.h>
#include <ros/package.h>
#include <stdlib.h>
#include <string>
#include <fstream>
#include <sstream> 
#include <stdexcept>

#define CHANGE_HW_VERSION_OK       0
#define CHANGE_HW_VERSION_FAIL    -1
#define CHANGE_HW_VERSION_NOT_ARM -2

int change_hardware_version_and_reboot(int old_version, int new_version);

#endif
