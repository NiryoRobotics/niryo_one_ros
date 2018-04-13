/*
    xl320_driver.h
    Copyright (C) 2017 Niryo
    All rights reserved.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef XL320_DRIVER_H
#define XL320_DRIVER_H

#include "dynamixel_sdk/dynamixel_sdk.h"
#include <vector>
#include <string>

#define PROTOCOL_VERSION 2.0
#define DXL_MODEL_NUMBER 350

#define DXL_LEN_ONE_BYTE  1
#define DXL_LEN_TWO_BYTES 2

// Table here : http://support.robotis.com/en/product/actuator/dynamixel_x/xl_series/xl-320.htm
#define DXL_ADDR_MODEL_NUMBER          0
#define DXL_ADDR_FIRMWARE_VERSION      2
#define DXL_ADDR_ID                    3
#define DXL_ADDR_BAUDRATE              4
#define DXL_ADDR_RETURN_DELAY_TIME     5
#define DXL_ADDR_CW_ANGLE_LIMIT        6  
#define DXL_ADDR_CCW_ANGLE_LIMIT       8                  // EEPROM
#define DXL_ADDR_CONTROL_MODE          11
#define DXL_ADDR_LIMIT_TEMPERATURE     12
#define DXL_ADDR_LOWER_LIMIT_VOLTAGE   13
#define DXL_ADDR_UPPER_LIMIT_VOLTAGE   14
#define DXL_ADDR_MAX_TORQUE            15
#define DXL_ADDR_RETURN_LEVEL          17
#define DXL_ADDR_ALARM_SHUTDOWN        18

#define DXL_ADDR_TORQUE_ENABLE         24
#define DXL_ADDR_LED                   25
#define DXL_ADDR_D_GAIN                27
#define DXL_ADDR_I_GAIN                28
#define DXL_ADDR_P_GAIN                29
#define DXL_ADDR_GOAL_POSITION         30
#define DXL_ADDR_GOAL_SPEED            32
#define DXL_ADDR_GOAL_TORQUE           35
#define DXL_ADDR_PRESENT_POSITION      37                  // RAM
#define DXL_ADDR_PRESENT_SPEED         39
#define DXL_ADDR_PRESENT_LOAD          41
#define DXL_ADDR_PRESENT_VOLTAGE       45
#define DXL_ADDR_PRESENT_TEMPERATURE   46
#define DXL_ADDR_REGISTERED            47
#define DXL_ADDR_MOVING                49
#define DXL_ADDR_HW_ERROR_STATUS       50                  
#define DXL_ADDR_PUNCH                 51

// Communication Result
//#define COMM_SUCCESS        0       // tx or rx packet communication success
//#define COMM_PORT_BUSY      -1000   // Port is busy (in use)                    
//#define COMM_TX_FAIL        -1001   // Failed transmit instruction packet      
//#define COMM_RX_FAIL        -1002   // Failed get status packet               
//#define COMM_TX_ERROR       -2000   // Incorrect instruction packet               FROM dynamixel_sdk
//#define COMM_RX_WAITING     -3000   // Now recieving status packet           
//#define COMM_RX_TIMEOUT     -3001   // There is no status packet            
//#define COMM_RX_CORRUPT     -3002   // Incorrect status packet             
//#define COMM_NOT_AVAILABLE  -9000 //                                      
//
/////////////////// Protocol 2.0 Error bit /////////////////
//#define ERRNUM_RESULT_FAIL      1       // Failed to process the instruction packet.  
//#define ERRNUM_INSTRUCTION      2       // Instruction error                          
//#define ERRNUM_CRC              3       // CRC check error                            
//#define ERRNUM_DATA_RANGE       4       // Data range error                           
//#define ERRNUM_DATA_LENGTH      5       // Data length error                          
//#define ERRNUM_DATA_LIMIT       6       // Data limit error                           
//#define ERRNUM_ACCESS 7 // Access error                                              

//#define ERRBIT_ALERT 128 //When the device has a problem, this bit is set to 1. Check "Device Status Check" value.

#define GROUP_SYNC_REDONDANT_ID     10
#define GROUP_SYNC_READ_RX_FAIL     11
#define LEN_ID_DATA_NOT_SAME        20

#define PING_WRONG_MODEL_NUMBER     30

#define DXL_FAIL_OPEN_PORT         -4500
#define DXL_FAIL_PORT_SET_BAUDRATE -4501
#define DXL_FAIL_SETUP_GPIO        -4502

class XL320Driver {

    private:
        dynamixel::PortHandler *portHandler;
        dynamixel::PacketHandler *packetHandler;

        std::string deviceName;
        int baudRate;
        
        int syncWrite1Byte  (uint8_t address, std::vector<uint8_t> &id_list, std::vector<uint32_t> &data_list);
        int syncWrite2Bytes (uint8_t address, std::vector<uint8_t> &id_list, std::vector<uint32_t> &data_list);

        int read1Byte       (uint8_t address, uint8_t id, uint32_t *data);
        int read2Bytes      (uint8_t address, uint8_t id, uint32_t *data);
        int syncRead        (uint8_t address, uint8_t data_len, std::vector<uint8_t> &id_list, std::vector<uint32_t> &data_list);

    public:
        XL320Driver(std::string deviceName, int baudrate);

        int init();

        int scan(std::vector<uint8_t> &id_list);
        int ping(uint8_t id);
        int reboot(uint8_t id);

        // eeprom write
        int changeId            (uint8_t id, uint8_t new_id);
        int changeBaudRate      (uint8_t id, uint32_t new_baudrate);
        int setReturnDelayTime  (uint8_t id, uint32_t return_delay_time);
        int setLimitTemperature (uint8_t id, uint32_t temperature);
        int setMaxTorque        (uint8_t id, uint32_t torque);
        int setReturnLevel      (uint8_t id, uint32_t return_level);
        int setAlarmShutdown    (uint8_t id, uint32_t alarm_shutdown);

        // eeprom read
        int readReturnDelayTime  (uint8_t id, uint32_t *return_delay_time);
        int readLimitTemperature (uint8_t id, uint32_t *limit_temperature);
        int readMaxTorque        (uint8_t id, uint32_t *max_torque);
        int readReturnLevel      (uint8_t id, uint32_t *return_level);
        int readAlarmShutdown    (uint8_t id, uint32_t *alarm_shutdown);

        // ram write --> return comm_result
        int setTorqueEnable   (uint8_t id, uint32_t torque_enable);
        int setLed            (uint8_t id, uint32_t led_value);
        int setGoalPosition   (uint8_t id, uint32_t position);
        int setGoalVelocity   (uint8_t id, uint32_t velocity);
        int setGoalTorque     (uint8_t id, uint32_t torque);

        int syncWriteLed          (std::vector<uint8_t> &id_list, std::vector<uint32_t> &led_list);
        int syncWriteTorqueEnable (std::vector<uint8_t> &id_list, std::vector<uint32_t> &torque_enable_list); 
        int syncWritePositionGoal (std::vector<uint8_t> &id_list, std::vector<uint32_t> &position_list);
        int syncWriteVelocityGoal (std::vector<uint8_t> &id_list, std::vector<uint32_t> &velocity_list);
        int syncWriteTorqueGoal   (std::vector<uint8_t> &id_list, std::vector<uint32_t> &torque_list);

        // ram read
        int readPosition       (uint8_t id, uint32_t *present_position);
        int readVelocity       (uint8_t id, uint32_t *present_velocity);
        int readLoad           (uint8_t id, uint32_t *present_load);
        int readTemperature    (uint8_t id, uint32_t *temperature);
        int readVoltage        (uint8_t id, uint32_t *voltage);
        int readHardwareStatus (uint8_t id, uint32_t *hardware_status);

        int syncReadPosition       (std::vector<uint8_t> &id_list, std::vector<uint32_t> &position_list);
        int syncReadVelocity       (std::vector<uint8_t> &id_list, std::vector<uint32_t> &velocity_list);
        int syncReadLoad           (std::vector<uint8_t> &id_list, std::vector<uint32_t> &load_list);
        int syncReadTemperature    (std::vector<uint8_t> &id_list, std::vector<uint32_t> &temperature_list);
        int syncReadVoltage        (std::vector<uint8_t> &id_list, std::vector<uint32_t> &voltage_list);
        int syncReadHwErrorStatus  (std::vector<uint8_t> &id_list, std::vector<uint32_t> &hw_error_list);
};

#endif
