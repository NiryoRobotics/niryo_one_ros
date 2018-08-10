# Niryo One ROS stack

(Niryo One : [https://niryo.com](https://niryo.com/?utm_source=github))

![niryo one rviz simulation](https://niryo.com/wp-content/uploads/2018/08/ros_rviz_niryo_one_colors.png)

This repository contains all ROS packages used on Niryo One (Raspberry Pi 3B - Xubuntu for ARM).

## How to use Niryo One with a graphical interface ?

You can [download Niryo One Studio](https://niryo.com/download/?utm_source=github) (Linux, Windows and MacOS compatible).

## How to install Niryo One ROS packages on your computer (x86) - Simulation Mode

Requirements :
* Ubuntu 16.04
* ROS kinetic  (other versions are not supported)

First install ROS kinetic "Desktop-Full" (tutorial [here](http://wiki.ros.org/kinetic/Installation/Ubuntu)).

You'll need to install some additional ROS packages :
```
sudo apt-get install ros-kinetic-robot-state-publisher ros-kinetic-moveit ros-kinetic-manipulation-msgs ros-kinetic-rosbridge-suite ros-kinetic-joy ros-kinetic-ros-control ros-kinetic-ros-controllers ros-kinetic-tf2-web-republisher
```
You'll also need to install an additional Python module :
```
sudo -H pip install jsonpickle
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
You can now launch Rviz with Niryo One (only display mode with some cursors to move the joints):
```
roslaunch niryo_one_description display.launch
```

You can also launch the complete Niryo One ROS Stack instead, which you can control from Niryo One Studio.
```
roslaunch niryo_one_bringup desktop_rviz_simulation.launch
```

The main differences between this launch file and the launch file executed on Raspberry Pi 3B (rpi\_setup.launch) is that the hardware functionalities are disabled, and you get a 3D simulation view with Rviz.

Note that Niryo One ROS packages have been developed with **ROS kinetic, on Ubuntu 16.04**. Other ROS versions and OS distributions are not supported.

## Niryo One ROS Stack overview

Here's a global overview of the Niryo One ROS Stack :

![niryo one ros stack - global overview](https://niryo.com/wp-content/uploads/2017/12/niryo_one_ros.png)

**You can find more specific and detailed info in each package's README.**

## Developer Documentation

* [Get started with the Niryo One Stack](https://niryo.com/docs/niryo-one/developer-tutorials/get-started-with-the-niryo-one-ros-stack/). This will help you understand the architecture and where to start as a developer.
* [C++ example to move the robot](https://github.com/smaassen/niryo_one_tester) by Steve Maassen
* [Python API](https://github.com/NiryoRobotics/niryo_one_ros/tree/master/niryo_one_python_api)
* [Remotely control Niry One (ROS multi-machines)](https://niryo.com/docs/niryo-one/developer-tutorials/remotely-control-niryo-one-ros-multi-machines/)

## Any question ?

If you have a question and you don't find the answer here or on our [documentation](https://niryo.com/docs/niryo-one/), please send us an email at support@niryo.com.

Thank you !
