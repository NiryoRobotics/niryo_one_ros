# Niryo One User Interface

This packages handles high-level user interface commands.

### Joystick interface

This interfaces uses the _joy_ ROS package to read data from a Xbox controller, and then sends commands to Niryo One joint trajectory controller. 

Each joint can be controlled separately with different buttons on the joystick.

### Blockly server

This is an actionlib server that can receive Blockly commands.

* Receives Blockly commands (Blockly XML)
* Handles concurrent requests
* Generates Python code from XML (through a nodejs server)
* Executes the generated Python code (corresponds to the Niryo One Python API functions)
* Returns appropriate status and message

**Why a nodejs server ?**

The Blockly library from Google is running with JavaScript, so we installed a nodejs server to handle the generation of Python code from XML.

**Installation**

After you install ROS packages and execute _catkin\_make_ you still have some installation steps if you want to use Blockly on your computer or Raspberry Pi 3.

1. Make sure that you have a recent nodejs version (not the default installed one)
* curl -sL https://deb.nodesource.com/setup_8.x | sudo -E bash -
* sudo apt-get install -y nodejs
2. Install node modules in the blockly\_code\_generator directory (where you can find package.json).
* npm install
3. Create an executable
* sudo npm link
