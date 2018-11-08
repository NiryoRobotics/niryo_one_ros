# Niryo One Python API

To use Python API :

1. Connect to Niryo One via ssh

2. Create a Python file

```
touch test_python_api.py
chmod +x test_python_api.py
```

3. Use this template to write your code

```python
#!/usr/bin/env python

from niryo_one_python_api.niryo_one_api import *
import rospy
import time
rospy.init_node('niryo_one_example_python_api')

n = NiryoOne()

try:
    # Your code here
except NiryoOneException as e:
    print e
    # Handle errors here
```

You can find some examples [here](https://github.com/NiryoRobotics/niryo_one_ros/tree/master/niryo_one_python_api/examples).

## Documentation

### Constants

You can use those predefined constants (instead of numbers) in the Python API methods. Please read the examples to see how to use them.

##### Tools IDs

* TOOL\_NONE
* TOOL\_GRIPPER\_1\_ID
* TOOL\_GRIPPER\_2\_ID
* TOOL\_GRIPPER\_3\_ID
* TOOL\_ELECTROMAGNET\_1\_ID
* TOOL\_VACUUM\_PUMP\_1\_ID
    
##### Digital pins

* PIN\_MODE\_OUTPUT
* PIN\_MODE\_INPUT
* PIN\_HIGH
* PIN\_LOW
* GPIO\_1A
* GPIO\_1B
* GPIO\_1C
* GPIO\_2A
* GPIO\_2B
* GPIO\_2C
* SW\_1
* SW\_2
  
##### For shift\_pose function

* AXIS\_X
* AXIS\_Y
* AXIS\_Z
* ROT\_ROLL
* ROT\_PITCH
* ROT\_YAW

### Class methods

##### calibrate\_auto

Calibrate robot motors automatically (by moving axis). If calibration is not needed, this method will do nothing.

##### calibrate\_manual

Calibrate robot motors manually (the robot just needs to be in 'home' position and to have been auto calibrated at least once). If calibration is not needed, this method will do nothing.

##### activate\_learning\_mode

Params: 
* activate (0 or 1)

Activate or deactivate learning mode (= disable/enable torque on motors).

##### move\_joints

Params:
* joints (array of 6 joints)

Move the arm with a joint command.

##### move\_pose

Params:
* position x (m)
* position y (m)
* position z (m)
* rotation x (rad)
* rotation y (rad)
* rotation z (rad)

Move the arm with a pose command.

##### shift\_pose

Params:
* axis (0: pos.x, 1: pos.y, 2: pos.z, 3: rot.x, 4: rot.y, 5: rot.z)
* value (m)

Move the arm by shifting the current pose on <axis> by <value>.

##### set\_arm\_max\_velocity

Params:
* percentage (1-100)

Set the arm max velocity scaling factor.

##### pin\_mode

Params:
* pin (GPIO number)
* mode (0: OUTPUT, 1: INPUT)

Set a digital I/O pin on INPUT or OUTPUT mode.

##### digital\_write

Params:
* pin (GPIO number)
* state (0: LOW, 1: HIGH)

Set a digital I/O pin to LOW or HIGH. Note that the pin must have been previously set as OUTPUT.

##### digital\_read

Params:
* pin (GPIO number)

Returns the current pin state (0: LOW, 1: HIGH).

##### change\_tool

Params:
* tool id (0 to detach current tool)

Change current attached tool. **Before you execute any action on a tool, you have to select it with this method.**

##### open\_gripper

Params:
* tool id
* open speed (between 0 and 1000, recommended : between 100 and 500)

Open gripper at selected speed.

##### close\_gripper

Params:
* tool id
* close speed (between 0 and 1000, recommended : between 100 and 500)

Close gripper at selected speed. The gripper will stop when it detects the object to grab.

##### pull\_air\_vacuum\_pump

Params:
* tool id

Activate vacuum pump (pick object).

##### push\_air\_vacuum\_pump

Params: 
* tool id

Deactivate vacuum pump (place object)

##### setup\_electromagnet

Params:
* tool id
* pin (GPIO number)

Setup electromagnet on digital I/O <pin> (set the pin mode to OUTPUT). You need to select and setup the electromagnet before using it.

##### activate\_electromagnet

Params:
* tool id
* pin (GPIO number)

Activate electromagnet on digital I/O <pin> (pick object). This will set the pin state to HIGH.

##### deactivate\_electromagnet

Params:
* tool id
* pin (GPIO number)

Deactivate electromagnet on digital I/O <pin> (place object). This will set the pin state to LOW.

##### get\_saved\_position\_list

Get all saved positions on the robot

##### wait

Params:
* time (seconds)

Blocks and wait for <time> seconds.

##### get\_joints

Returns an array containing the current angles for all 6 axis (in radian).

##### get\_pose

Returns a RobotState object (see in niryo\_one\_msgs package) containing the pose (position in meters + orientation in radian) of the end effector tool.

##### get\_hardware\_status

Returns a HardwareStatus object (see in niryo\_one\_msgs package) containing useful info about the motors state, connection, temperature, etc. (temperature unit: °C)

##### get\_learning\_mode

Returns a boolean that indicates whether learning mode is activated or not.

##### get\_digital\_io\_state

Returns a DigitalIOState object (see in niryo\_one\_msgs package) containing information (mode: input or output + state: high or low) for all the 6\* 5V digital pins + 2\* 12V switches.
