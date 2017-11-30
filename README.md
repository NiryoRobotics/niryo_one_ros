# Niryo One ROS stack

(Niryo One : [https://niryo.com](https://niryo.com/?utm_source=github))

--- DOCUMENTATION IN PROGRESS, WILL BE UPDATE WITH MORE INFO ---

This repository contains all ROS packages used on Niryo One (Raspberry Pi 3 - Xubuntu for ARM).

## How to install Niryo One ROS packages on your computer (x86)

Requirements :
* Ubuntu 16.04
* ROS kinetic  (other versions are not supported)

First install ROS kinetic "Desktop-Full" (tutorial [here](http://wiki.ros.org/kinetic/Installation/Ubuntu))
You'll need to install some additional ROS packages :
```
sudo apt-get install ros-kinetic-robot-state-publisher ros-kinetic-moveit ros-kinetic-manipulation-msgs ros-kinetic-rosbridge-suite ros-kinetic-joy ros-kinetic-ros-control ros-kinetic-ros-controllers
```
Create a catkin workspace and clone Niryo One ROS stack :
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone https://github.com/NiryoRobotics/niryo_one_ros.git .
```
Build the packages :
```
cd ~/catkin_ws
catkin_make
```
Don't forget to use those commands before you try to launch anything (you can add them in your .bashrc) :
```
source /opt/ros/kinetic/setup.bash
source ~/catkin_ws/devel/setup.bash
```
You can now launch Rviz with Niryo One :
```
roslaunch niryo_one_description display.launch
```
-  If you find a problem please send us a message (support@niryo.com) so we can quickly correct any mistake


Niryo One ROS packages have been developed with **ROS kinetic, on Ubuntu 16.04**. Other ROS versions and OS distributions are not supported.


## How to use Niryo One with a graphical interface ?

You can download Niryo desktop software (Linux, Windows and MacOS compatible) : _coming soon..._

## Niryo One full documentation

_coming soon..._

COMING SOON :
- Instructions to run ROS packages on Niryo One Raspberry Pi 3 official image (Xubuntu)
- Code diagrams for a better understanding of all the packages and dependencies
- Detailed description of all packages
- Code examples to use the Niryo One Python API


Packages forked from other repositories :
- dynamixel_sdk : added compatibility with custom Niryo Raspberry Pi 3 shield
- mcp_can (from MCP_CAN, an Arduino library) : made compatible with Raspberry Pi 3
