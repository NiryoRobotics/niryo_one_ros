#!/usr/bin/env python

# niryo_one_button.py
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
import RPi.GPIO as GPIO
import subprocess

from niryo_one_rpi.shutdown import * 

from niryo_one_msgs.srv import SetInt

# todo include led_state constant

BUTTON_GPIO = 4

class NiryoButton:
    
    def read_value(self):
        return GPIO.input(self.pin)

    def __init__(self):
        self.pin = BUTTON_GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        rospy.loginfo("GPIO setup : " + str(self.pin) + " as input")
        rospy.sleep(0.1)

        self.last_state = self.read_value()
        self.consecutive_pressed = 0
        self.activated = True
        
        self.timer_frequency = 20.0
        self.shutdown_action = False
        self.hotspot_action = False

        self.button_timer = rospy.Timer(rospy.Duration(1.0/self.timer_frequency), self.check_button)
        rospy.on_shutdown(self.shutdown)
        rospy.loginfo("Niryo One Button started")

    def shutdown(self):
        rospy.loginfo("Shutdown button, cleanup GPIO")
        self.button_timer.shutdown()

    def send_hotspot_command(self):
        rospy.loginfo("HOTSPOT")
        send_led_state(5)
        rospy.wait_for_service('/niryo_one/wifi/set_hotspot')
        try:
           set_hotspot = rospy.ServiceProxy('/niryo_one/wifi/set_hotspot', SetInt)
           set_hotspot()
        except rospy.ServiceException, e:
            rospy.logwarn("Could not call set_hotspot service")

    def send_trigger_sequence_autorun(self):
        rospy.loginfo("Trigger sequence autorun from button")
        try:
            rospy.wait_for_service('/niryo_one/sequences/trigger_sequence_autorun', 0.1)
            trigger = rospy.ServiceProxy('/niryo_one/sequences/trigger_sequence_autorun', SetInt)
            trigger(1) # value doesn't matter, it will switch state on the server
        except (rospy.ServiceException, rospy.ROSException), e:
            return

    def check_button(self, event):
        if not self.activated:
            return

        # Execute action if flag True
        if self.hotspot_action:
            self.send_hotspot_command()
            self.hotspot_action = False
            self.shutdown_action = False
        elif self.shutdown_action:
            send_shutdown_command()
            self.hotspot_action = False
            self.shutdown_action = False

        # Read button state
        state = self.read_value()

        # Check if there is an action to do
        if state == 0:
            self.consecutive_pressed += 1
        elif state == 1: # button released
            if self.consecutive_pressed > self.timer_frequency * 20:
                self.activated = False # deactivate button if pressed more than 20 seconds
            elif self.consecutive_pressed > self.timer_frequency * 6:
                self.hotspot_action = True
            elif self.consecutive_pressed > self.timer_frequency * 3:
                self.shutdown_action = True
            elif self.consecutive_pressed >= 1:
                self.send_trigger_sequence_autorun()
            self.consecutive_pressed = 0
            
        # Use LED to help user know which action to execute
        if self.consecutive_pressed > self.timer_frequency * 20:
            send_led_state(1)
        elif self.consecutive_pressed > self.timer_frequency * 6:
            send_led_state(5)
        elif self.consecutive_pressed > self.timer_frequency * 3:
            send_led_state(1)

