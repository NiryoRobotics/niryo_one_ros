# niryo_one_python_tcp_client
Tcp client that communicates with the tcp server of the Niryo One

### Connection

Port of the server: 40001

### Functions available

All enums are registered in the Command.py file.

* `connect(ip_address)`
    * ip_address: ip address of the robot
        * **String** type (`"10.10.10.10"`, ...)

    This function connect to the robot on the given ip_address.
    Return `True` on success or `False` on failure (with error printed)

* `calibrate(mode)`
    * mode: calibrate in automatic mode or manual
        * **CalibrateMode** enum (`AUTO` / `MANUAL`)

    Calibrate robot motors according to the mode.
    <br>Automatic: moving each axis.
    <br>Manual: set the position as 'current calibration home position'.
    <br>If calibration is not needed, this method will do nothing.

* `set_learning_mode(enabled)`
    * enabled: enable the learning mode on True / disable on False
        * **Boolean** (`True` / `False`)

    Activate or deactivate learning mode (= disable/enable torque on motors).

* `move_joints(j1, j2, j3, j4, j5, j6)`
    * j1 / j2 / j3 / j4 / j5 / j6: value (radian) for each joint
        * **float** type only

    Move the arm with a joint command.

* `move_pose(x, y, z, roll, pitch, yaw)`
    * x, y, z: value (meter) for the corresponding axis position
        * **float** type only
    * roll, pitch, yaw: value (radian) for the corresponding axis rotation
        * **float** type only

    Move the arm with a pose command.

* `shift_pose(axis, shift_value)`
    * axis: on which axis the position / rotation will be shifted
        * **RobotAxis** enum (`X`, `Y`, `Z`, `ROLL`, `PITCH`, `YAW`)
    * shift_value: value to shift the axis (radian or meter depending of axis, see move_pose)
        * **float** type

    Move the arm by shifting the current pose on 'axis' by 'shift_value'.

* `set_arm_max_velocity(percentage)`
    * percentage: percetage of max speed applied
        * **integer** type from 1 to 100

    Set the arm max velocity scaling factor.

* `set_joystick_mode(enabled)`
    * enabled: enable the joystick mode on True / disable on False
        * **Boolean** (`True` / `False`)

    Enable or disable joystick mode (control the robot with a joystick controller).

* `set_pin_mode(pin, pin_mode)`
    * pin: which pin will be changed of mode
        * **RobotPin** enum type (`GPIO_1A`, `GPIO_1B`, `GPIO_1C`, `GPIO_2A`, `GPIO_2B`, `GPIO_2C`)
    * pin_mode: to which mode the pin should be changed
        * **PinMode** enum type (`INPUT`, `OUTPUT`)

    Set a digital I/O pin on INPUT or OUTPUT mode.

* `digital_write(pin, digital_state)`
    * pin: which pin will have it's state modifed
        * **RobotPin** enum type (`GPIO_1A`, `GPIO_1B`, `GPIO_1C`, `GPIO_2A`, `GPIO_2B`, `GPIO_2C`)
    * digital_state: to which state the pin should be changed
        * **DigitalState** enum type (`LOW`, `HIGH`)

    Set a digital I/O pin to LOW or HIGH. Note that the pin must have been previously set as OUTPUT.

* `digital_read(pin)`
    * pin: from which pin we will read it's state
        * **RobotPin** enum type (`GPIO_1A`, `GPIO_1B`, `GPIO_1C`, `GPIO_2A`, `GPIO_2B`, `GPIO_2C`)

    Returns the current pin state (0: LOW, 1: HIGH).

* `change_tool(tool)`
    * tool: which tool we want to use now (or `NONE` to disable)
        * **RobotTool** enum type (`NONE`, `GRIPPER_1`, `GRIPPER_2`, `GRIPPER_3`, `ELECTROMAGNET_1`, `VACUUM_PUMP_1`)

    Change current attached tool. **Before you execute any action on a tool, you have to select it with this method.**

* `open_gripper(gripper, speed)`
    * gripper: which gripper type is used now
        * **RobotTool** enum type (`GRIPPER_1`, `GRIPPER_2`, `GRIPPER_3`)
    * speed: at which speed we want to open the gripper
        * **integer** type (between 0 and 1000, recommended : between 100 and 500)

    Open gripper at selected speed.

* `close_gripper(gripper, speed)`
    * gripper: which gripper type is used now
        * **RobotTool** enum type (`GRIPPER_1`, `GRIPPER_2`, `GRIPPER_3`)
    * speed: at which speed we want to open the gripper
        * **integer** type (between 0 and 1000, recommended : between 100 and 500)

    Close gripper at selected speed. The gripper will stop when it detects the object to grab.

* `pull_air_vacuum_pump(vacuum_pump)`
    * vacuum_pump: which vacuum pump type is used now
        * **RobotTool** enum type (`VACUUM_PUMP_1`)

    Activate vacuum pump (pick object).

* `push_air_vacuum_pump(vacuum_pump)`
    * vacuum_pump: which vacuum pump type is used now
        * **RobotTool** enum type (`VACUUM_PUMP_1`)

    Deactivate vacuum pump (place object)

* `setup_electromagnet(electromagnet, pin)`
    * electromagnet: which electromagnet type is used now
        * **RobotTool** enum type (`ELECTROMAGNET_1`)
    * pin: on which pin the electromagnet is connected
        * **RobotPin** enum type (`GPIO_1A`, `GPIO_1B`, `GPIO_1C`, `GPIO_2A`, `GPIO_2B`, `GPIO_2C`)* `activate_electromagnet(electromagnet, pin)`

    Setup electromagnet on digital I/O <pin> (set the pin mode to OUTPUT). You need to select and setup the electromagnet before using it.

* `activate_electromagnet(electromagnet, pin)`
    * electromagnet: which electromagnet type is used now
        * **RobotTool** enum type (`ELECTROMAGNET_1`)
    * pin: on which pin the electromagnet is connected
        * **RobotPin** enum type (`GPIO_1A`, `GPIO_1B`, `GPIO_1C`, `GPIO_2A`, `GPIO_2B`, `GPIO_2C`)* `activate_electromagnet(electromagnet, pin)`

    Activate electromagnet on digital I/O <pin> (pick object). This will set the pin state to HIGH.

* `deactivate_electromagnet(electromagnet, pin)`
    * electromagnet: which electromagnet type is used now
        * **RobotTool** enum type (`ELECTROMAGNET_1`)
    * pin: on which pin the electromagnet is connected
        * **RobotPin** enum type (`GPIO_1A`, `GPIO_1B`, `GPIO_1C`, `GPIO_2A`, `GPIO_2B`, `GPIO_2C`)* `activate_electromagnet(electromagnet, pin)`

    Deactivate electromagnet on digital I/O <pin> (place object). This will set the pin state to LOW.

* `get_joints()`

    Returns an array containing the current angles for all 6 axis (in radian).

* `get_pose()`

    Returns a RobotState object (see [the niryo one ros object package documentation](https://github.com/NiryoRobotics/niryo_one_ros/tree/master/niryo_one_msgs/msg)) containing the pose (position in meters + orientation in radian) of the end effector tool.

* `get_hardware_status()`

    Returns a HardwareStatus object (see [the niryo one ros object package documentation](https://github.com/NiryoRobotics/niryo_one_ros/tree/master/niryo_one_msgs/msg)) containing useful info about the motors state, connection, temperature, etc. (temperature unit: °C)

* `get_learning_mode()`

    Returns a boolean that indicates whether learning mode is activated or not.

* `get_digital_io_state()`

    Returns a DigitalIOState object (see [the niryo one ros object package documentation](https://github.com/NiryoRobotics/niryo_one_ros/tree/master/niryo_one_msgs/msg)) containing information (mode: input or output + state: high or low) for all the 6\* 5V digital pins + 2\* 12V switches.
