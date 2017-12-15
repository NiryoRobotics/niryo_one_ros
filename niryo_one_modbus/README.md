# Niryo One Modbus/TCP Server

The Modbus/TCP server is running on port 5020 by default.
It has been built on top of the [pymodbus](http://pymodbus.readthedocs.io/en/latest/index.html) library.
This enables you to make Niryo One communicate with a PLC, or another computer in the same network.

All 4 Modbus datastores are implemented : _Coil_, _Discrete Input_, _Holding Register_, _Input Register_. Each datastore has a different set of functionalities. Note that **each datastore contains a completely different set of data**.

_Discrete Input_ and _Input register_ are READ-ONLY tables. For Niryo One those have been used to keep the robot state.
_Coil_ and _Holding Register_ are READ/WRITE tables. For Niryo One those have been used to give user commands to the robot. Hence, those 2 tables do not contain the robot state, but the last given command.

Address tables start at 0.

## Coil

Each address contains a 1bit value.
READ/WRITE (the stored values correspond to the last given command, not the current robot state)
Accepted Modbus functions :
  * 0x01: READ_COILS
  * 0x05: WRITE_SINGLE_COIL

This datastore can be used to set Digital I/O mode and state. Digital I/O numbers used for Modbus:
* 0 : 1A
* 1 : 1B
* 2 : 1C
* 3 : 2A
* 4 : 2B
* 5 : 2C

|Address| Description |
|-------|-------------|
| 0-5 | Digital I/O mode (Input = 1, Output = 0) |
| 100-105 | Digital I/O state (High = 1, Low = 0) |
| 200-299 | Can be used to store your own variables |

## Discrete Input

Each address contains a 1bit value.
READ-ONLY
Accepted Modbus functions :
* 0x02: READ_DISCRETE_INPUTS

This datastore can be used to read Digital I/O mode and state. See _Coil_ section above for digital I/O number mapping.

|Address| Description |
|-------|-------------|
| 0-5 | Digital I/O mode (Input = 1, Output = 0) |
| 100-105 | Digital I/O state (High = 1, Low = 0) |

## Holding Register

Each address contains a 16bit value.
READ/WRITE (the stored values correspond to the last given command, not the current robot state)
Accepted Modbus functions :
* 0x03: READ_HOLDING_REGISTERS
* 0x06: WRITE_SINGLE_REGISTER

|Address| Description |
|-------|-------------|
| 200-299 | Can be used to store your own variables |
| 300 | Learning Mode (On = 1, Off = 0) |
| 301 | Joystick Enabled (On = 1, Off = 0) |


## Input Register

Each address contains a 16bit value.
READ-ONLY
Accepted Modbus functions :
* 0x04: READ_INPUT_REGISTERS

|Address| Description |
|-------|-------------|
| 0-5 | Joints 1-6 (mrad) |
| 10 | Position x |
| 11 | Position y |
| 12 | Position z |
| 13 | Orientation x |
| 14 | Orientation y |
| 15 | Orientation z |
| 200 | Selected tool ID (0 for no tool) |
| 300 | Learning Mode activated |
| 301 | Joystick enabled |
| 400 | Motors connection up (Ok = 1, Not ok = 0) |
| 401 | Calibration needed flag |
| 402 | Calibration in progress flag |
| 403 | Raspberry Pi 3 temperature |
| 404 | Raspberry Pi 3 available disk size |
| 405 | Raspberry Pi 3 ROS log size |
| 406 | Niryo One RPI image version n.1 |
| 407 | Niryo One RPI image version n.2 |
| 408 | Niryo One RPI image version n.3 |

## Connect to the Modbus/TCP server with Python, as a client :

You can test the Modbus/TCP server, for example from a remote computer on the same network.

```python
#!/usr/bin/env python
from pymodbus.client.sync import ModbusTcpClient # you need to "pip install pymodbus" 

address = 'insert the Modbus/TCP server IP address here'
client = ModbusTcpClient(address, port=5020)
client.connect()

# Your code here
# Check out the pymodbus documentation : http://pymodbus.readthedocs.io/en/latest/index.html

client.close()
```
