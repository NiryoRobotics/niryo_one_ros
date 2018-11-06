# Niryo One Debug

This package provides tools to debug parts of Niryo One.

## send\_custom\_dxl\_value.py

Use this Python script to set a motor register value (only during Niryo One runtime).

Works on Raspberry Pi 3 only.

--> Useful to change RAM values at runtime.

```
cd scripts
./send_custom_dxl_value.py --help 
```

## dxl\_debug\_tools (Cpp)

Use this tool **ONLY WHEN NIRYO ONE ROS STACK IS NOT RUNNING** to scan motors, change motor id, baudrate, and any other register
 
Works on Raspberry Pi 3 only.

--> Useful to setup a new Dynamixel motor (ID and baudrate), do advanced debugging, and set EEPROM values.

Make sure you have compiled the C++ file before:

```
cd ~/catkin_ws
catkin_make -j2
cd ~/catkin_ws/devel/lib/niryo_one_debug
./dxl_debug_tools --help
```

## Where to find Dynamixel registers?

* [XL-320 doc](http://emanual.robotis.com/docs/en/dxl/x/xl320/#control-table-of-eeprom-area)
* [XL-430 doc](http://emanual.robotis.com/docs/en/dxl/x/xl430-w250/#control-table-of-eeprom-area)
