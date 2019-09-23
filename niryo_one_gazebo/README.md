# Niryo One Gazebo Simulation

This package provides config and launch files to run Niryo One in Gazebo.

Developed and tested on ROS Melodic/Gazebo 9.

## Start the Gazebo simulation

First start Gazebo (empty world) and the model of Niryo One:
```
roslaunch niryo_one_gazebo niryo_one_world.launch
```
Then, start the controllers (ros_control) and the Moveit interface:
```
roslaunch niryo_one_gazebo niryo_one_control.launch
```

You can find the URDF used for this simulation [here](https://github.com/NiryoRobotics/niryo_one_ros/blob/gazebo_simulation/niryo_one_description/urdf/v2/gazebo_niryo_one.urdf.xacro).

## Control the robot

The ROS interface to control Niryo One is the same for the real robot and the Gazebo simulation.

The controller used for Niryo One is a joint\_trajectory\_controller (from ros\_control). See the [joint\_trajectory\_controller documentation](http://wiki.ros.org/joint_trajectory_controller) to get more info.

You can also use the MoveGroup interface - Python or Cpp - from Moveit (higher level). This interface will call the Moveit motion planning functionality and then send the computed plan to the joint\_trajectory\_controller.
