### Connection

Port of the server: 40001

### Communication

* Only one client can communicate with the server (reconnection effective but no multi clients).
* The server answer only after the command is done, so it cannot deal with multiple commands at the same time.

### Format

For easier usage and easier debugging, the communication is in ascii format.

* For command without parameter, the format is:
    * `COMMAND`
* For command with parameter, the format is:
    * `COMMAND: param1, param2`
* All commands / parameters are case insensitive and are spaces tolerant
    * Example: `command:PARAM1`
* For all **active** commands (setter, move, ...), the result look like this:
    * `COMMAND: [OK / KO]`
        * Example: `OPEN_GRIPPER: KO`
    * **On error**: A message displaying the error that occurred

### Commands

* "CALIBRATE"
* "SET_LEARNING_MODE"
* "MOVE_JOINTS"
* "MOVE_POSE"
* "SHIFT_POSE"
* "SET_ARM_MAX_VELOCITY"
* "SET_JOYSTICK_MODE"
* "SET_PIN_MODE"
* "DIGITAL_WRITE"
* "DIGITAL_READ"
* "CHANGE_TOOL"
* "OPEN_GRIPPER"
* "CLOSE_GRIPPER"
* "PULL_AIR_VACUUM_PUMP"
* "PUSH_AIR_VACUUM_PUMP"
* "SETUP_ELECTROMAGNET"
* "ACTIVATE_ELECTROMAGNET"
* "DEACTIVATE_ELECTROMAGNET"
* "GET_SAVED_POSITION_LIST"
* "WAIT"
* "GET_JOINTS"
* "GET_POSE"
* "GET_HARDWARE_STATUS"
* "GET_LEARNING_MODE"
* "GET_DIGITAL_IO_STATE"

#### Calibrate

Parameters:
* Mode:
    * AUTO for an automatic calibration
    * MANUAL for a manual calibration

Example: `CALIBRATE: AUTO`

#### Set_learning_mode

Parameters:
* enabled:
    * TRUE to activate learning mode
    * FALSE to deactivate learning mode

Example:
    `SET_LEARNING_MODE: TRUE`

#### MOVE_JOINTS

Parameters:
* j1: value of joints in **float**
* j2: value of joints in **float**
* j3: value of joints in **float**
* j4: value of joints in **float**
* j5: value of joints in **float**
* j6: value of joints in **float**

Example:
    `MOVE_JOINTS: 0.03, 0.0123, 0.456, 0.987, 0.654, 0.321`

#### MOVE_POSE

Parameters:
* x: value of x position **(m)** in **float**
* y: value of y position **(m)** in **float**
* z: value of z position **(m)** in **float**
* roll: value of roll rotation **(rad)** in **float**
* pitch: value of pitch rotation **(rad)** in **float**
* yaw: value of yaw rotation **(rad)** in **float**

Example:
    `MOVE_JOINTS: 0.03, 0.0123, 0.456, 0.987, 0.654, 0.321`

#### SHIFT_POSE

Parameters:
* axis: X / Y / Z / ROLL / PITCH / YAW
* value: value to shift the desired axis in **float**
    * Notes: Same units as before (m / rad) depending of axis

Example:
    `SHIFT_POSE: ROLL, 0.03142`

#### SET_ARM_MAX_VELOCITY

Parameters:
* percentage: percentage of max velocity in **integer**

Example:
    `SET_ARM_MAX_VELOCITY: 50`

#### SET_JOYSTICK_MODE

Parameters:
* enabled: TRUE / FALSE

Example:
    `SET_JOYSTICK_MODE: FALSE`

#### SET_PIN_MODE

Parameters:
* pin: GPIO_1A / GPIO_1B / GPIO_1C / GPIO_2A / GPIO_2B / GPIO_2C
* pin_mode: OUTPUT / INPUT

Example:
    `SET_PIN_MODE: GPIO_2B, OUTPUT`

#### DIGITAL_WRITE

Parameters:
* pin: GPIO_1A / GPIO_1B / GPIO_1C / GPIO_2A / GPIO_2B / GPIO_2C
* pin_state: LOW / HIGH

Example:
    `DIGITAL_WRITE: GPIO_2A, LOW`

#### DIGITAL_READ

Parameters:
* pin: GPIO_1A / GPIO_1B / GPIO_1C / GPIO_2A / GPIO_2B / GPIO_2C

Example:
    `DIGITAL_READ: GPIO_1A`

#### CHANGE_TOOL

Parameters:
* tool: GRIPPER_1 / GRIPPER_2 / GRIPPER_3 / VACUUM_PUMP_1 / ELECTROMAGNET_1

Example:
    `CHANGE_TOOL: GRIPPER_2`

#### OPEN_GRIPPER

Parameters:
* gripper_type: GRIPPER_1 / GRIPPER_2 / GRIPPER_3
* speed: speed value as **integer**

Example:
    `OPEN_GRIPPER: GRIPPER_3`

#### CLOSE_GRIPPER

Parameters:
* gripper_type: GRIPPER_1 / GRIPPER_2 / GRIPPER_3
* speed: speed value as **integer**

Example:
    `CLOSE_GRIPPER: GRIPPER_1`

#### PULL_AIR_VACUUM_PUMP

Parameters:
* vacuum_type: VACUUM_PUMP_1
    * Notes: Only one type available for now

Example:
    `PULL_AIR_VACUUM_PUMP: VACUUM_PUMP_1`

#### PUSH_AIR_VACUUM_PUMP

Parameters:
* vacuum_type: VACUUM_PUMP_1
    * Notes: Only one type available for now

Example:
    `PUSH_AIR_VACUUM_PUMP: VACUUM_PUMP_1`

#### SETUP_ELECTROMAGNET

Parameters:
* electromagnet_type: ELECTROMAGNET_1
    * Notes: Only one type available for now
* pin: GPIO_1A / GPIO_1B / GPIO_1C / GPIO_2A / GPIO_2B / GPIO_2C

Example:
    `SETUP_ELECTROMAGNET: ELECTROMAGNET_1`

#### ACTIVATE_ELECTROMAGNET

Parameters:
* electromagnet_type: ELECTROMAGNET_1
    * Notes: Only one type available for now
* pin: GPIO_1A / GPIO_1B / GPIO_1C / GPIO_2A / GPIO_2B / GPIO_2C

Example:
    `ACTIVATE_ELECTROMAGNET: ELECTROMAGNET_1, GPIO_2B`

#### DEACTIVATE_ELECTROMAGNET

Parameters:
* electromagnet_type: ELECTROMAGNET_1
    * Notes: Only one type available for now
* pin: GPIO_1A / GPIO_1B / GPIO_1C / GPIO_2A / GPIO_2B / GPIO_2C

Example:
    `DEACTIVATE_ELECTROMAGNET: ELECTROMAGNET_1, GPIO_2C`

#### WAIT

Parameters:
* how_much_time: time to sleep in **integer**
    * Notes: This is not a real time sleep but a ros sleep

Example:
    `WAIT: 5`

#### GET_SAVED_POSITION_LIST

No parameters

Example:
    `GET_SAVED_POSITION_LIST`

#### GET_JOINTS

No parameters

Example:
    `GET_JOINTS`

#### GET_POSE

No parameters

Example:
    `GET_POSE`

#### GET_HARDWARE_STATUS

No parameters

Example:
    `GET_HARDWARE_STATUS`

#### GET_LEARNING_MODE

No parameters

Example:
    `GET_LEARNING_MODE`

#### GET_DIGITAL_IO_STATE

No parameters

Example:
    `GET_DIGITAL_IO_STATE`