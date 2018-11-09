#!/usr/bin/env python

# rpi_ros_utils.py
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



import rospy
import subprocess

from niryo_one_msgs.srv import SetInt

def send_hotspot_command():
    rospy.loginfo("HOTSPOT")
    send_led_state(5)
    rospy.wait_for_service('/niryo_one/wifi/set_hotspot')
    try:
        set_hotspot = rospy.ServiceProxy('/niryo_one/wifi/set_hotspot', SetInt)
        set_hotspot()
    except rospy.ServiceException, e:
        rospy.logwarn("Could not call set_hotspot service")

def send_trigger_sequence_autorun():
    rospy.loginfo("Trigger sequence autorun from button")
    try:
        rospy.wait_for_service('/niryo_one/sequences/trigger_sequence_autorun', 0.1)
        trigger = rospy.ServiceProxy('/niryo_one/sequences/trigger_sequence_autorun', SetInt)
        trigger(1) # value doesn't matter, it will switch state on the server
    except (rospy.ServiceException, rospy.ROSException), e:
        return

def send_shutdown_command():
    rospy.loginfo("SHUTDOWN")
    send_led_state(1)
    rospy.loginfo("Activate learning mode")
    try:
        rospy.wait_for_service('/niryo_one/activate_learning_mode', 1)
    except rospy.ROSException, e:
        pass
    try:
        activate_learning_mode = rospy.ServiceProxy('/niryo_one/activate_learning_mode', SetInt)
        activate_learning_mode(1)
    except rospy.ServiceException, e:
        pass
    rospy.loginfo("Command 'sudo shutdown now'")
    try: 
        output = subprocess.check_output(['sudo', 'shutdown', 'now'])
    except subprocess.CalledProcessError:
        rospy.loginfo("Can't exec shutdown cmd")

def send_reboot_command():
    rospy.loginfo("REBOOT")
    send_led_state(1)
    rospy.loginfo("Activate learning mode")
    try:
        rospy.wait_for_service('/niryo_one/activate_learning_mode', 1)
    except rospy.ROSException, e:
        pass
    try:
        activate_learning_mode = rospy.ServiceProxy('/niryo_one/activate_learning_mode', SetInt)
        activate_learning_mode(1)
    except rospy.ServiceException, e:
        pass
    rospy.loginfo("Command 'sudo reboot'")
    try: 
        output = subprocess.check_output(['sudo', 'reboot'])
    except subprocess.CalledProcessError:
        rospy.loginfo("Can't exec reboot cmd")

def send_led_state(state):
    rospy.wait_for_service('/niryo_one/rpi/set_led_state')
    try:
        set_led = rospy.ServiceProxy('/niryo_one/rpi/set_led_state', SetInt)
        set_led(state)
    except rospy.ServiceException, e:
        rospy.logwarn("Could not call set_led_state service")

