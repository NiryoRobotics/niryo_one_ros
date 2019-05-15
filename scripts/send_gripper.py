#!/usr/bin/python
#
# Send a value to change the opening of the Robotiq gripper using an action
#

import argparse

import rospy
import actionlib
import control_msgs.msg
from std_msgs.msg import Float64


from ur_msgs.srv import SetPayload, SetIO
from ur_msgs.msg import *



def gripper_client_2(value):

    p1 = rospy.Publisher('/gripper_left_controller/command', Float64, queue_size=1)
    p2 = rospy.Publisher('/gripper_right_controller/command', Float64, queue_size=1)
    rospy.sleep(0.5)
    p1.publish(-value)
    p2.publish(value)



if __name__ == '__main__':
    try:
        # Get the angle from the command line
        parser = argparse.ArgumentParser()
        parser.add_argument("--value", type=float, default="0.2",
                            help="Value betwewen 0.2 (open) 0 (closed)")
        args = parser.parse_args()
        gripper_value = args.value
        # Start the ROS node
        rospy.init_node('gripper_command')
        # Set the value to the gripper
        result = gripper_client(gripper_value)
        
    except rospy.ROSInterruptException:
        print ("Program interrupted before completion")
