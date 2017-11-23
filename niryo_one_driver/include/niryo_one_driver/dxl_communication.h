/*
    dxl_communication.h
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

#ifndef DXL_COMMUNICATION_H
#define DXL_COMMUNICATION_H

#include <boost/shared_ptr.hpp>
#include <ros/ros.h>
#include <string>
#include <thread>

#include "niryo_one_driver/dxl_motor_state.h"
#include "niryo_one_driver/xl320_driver.h"

// we stop at 1022 instead of 1023, to get an odd number of positions (1023)
// --> so we can get a middle point (511)
#define DXL_TOTAL_ANGLE          296.67
#define DXL_MAX_POSITION         1022
#define DXL_MIN_POSITION         0
#define DXL_MIDDLE_POSITION      511
#define DXL_TOTAL_RANGE_POSITION 1023
    
#define RADIAN_TO_DEGREE 57.295779513082320876798154814105

#define TIME_TO_WAIT_IF_BUSY 0.0005

#define DXL_SCAN_OK                0
#define DXL_SCAN_MISSING_MOTOR    -50 
#define DXL_SCAN_UNALLOWED_MOTOR  -51

#define DXL_GRIPPER_ACTION_TIMEOUT 5

#define DXL_CONTROL_MODE_POSITION 1
#define DXL_CONTROL_MODE_VELOCITY 2
#define DXL_CONTROL_MODE_TORQUE   3

class DxlCommunication {

    public:
        
        DxlCommunication();
        int init();

        void startHardwareControlLoop(bool limited_mode);
        void stopHardwareControlLoop();

        void getCurrentPosition(double *axis_5_pos, double *axis_6_pos); 
        
        void getHardwareStatus(bool *is_connection_ok, std::string &error_message,
                int *calibration_needed, bool *calibration_in_progress,
                std::vector<std::string> &motor_names, std::vector<int32_t> &temperatures,
                std::vector<double> &voltages, std::vector<int32_t> &hw_errors);
        bool isConnectionOk();
        bool isOnLimitedMode();

        void setControlMode(int control_mode); // position, velocity, or torque
        void setGoalPosition(double axis_5_pos, double axis_6_pos);
        void setTorqueOn(bool on);
        void setLeds(std::vector<int> &leds);

        int scanAndCheck();
        

        // Dxl Tools
        void setTool(uint8_t id, std::string name);
        int pingAndSetTool(uint8_t id, std::string name);
        
        int openGripper(uint8_t id, uint16_t open_position, uint16_t open_speed, uint16_t open_hold_torque);
        int closeGripper(uint8_t id, uint16_t close_position, uint16_t close_speed, uint16_t close_hold_torque, uint16_t close_max_torque);
        
        int pullAirVacuumPump(uint8_t id, uint16_t pull_air_position, uint16_t pull_air_hold_torque);
        int pushAirVacuumPump(uint8_t id, uint16_t push_air_position);

    private:

        std::string device_name;
        int uart_baudrate;
       
        boost::shared_ptr<XL320Driver> xl320;

        std::vector<uint8_t> niryo_one_motors_ids;
        std::vector<uint8_t> allowed_motors_ids;

        uint16_t rad_pos_to_dxl_pos(double position_rad);
        double   dxl_pos_to_rad_pos(uint16_t position_dxl);

        void hardwareControlLoop();
        void hardwareControlRead();
        void hardwareControlWrite();

        void resetHardwareControlLoopRates();

        boost::shared_ptr<std::thread> hardware_control_loop_thread;

        // motors 
        DxlMotorState m5_1;
        DxlMotorState m5_2;
        DxlMotorState m6;
        DxlMotorState tool;

        // for hardware control
        
        bool is_dxl_connection_ok;
        std::string debug_error_message;

        uint8_t torque_on; // torque is ON/OFF for all motors at the same time
        
        bool is_tool_connected;

        bool hw_control_loop_keep_alive;
        bool hw_is_busy;
        bool hw_limited_mode;

        double hw_control_loop_frequency;

        int hw_fail_counter_read;

        double time_hw_data_last_write;
        double time_hw_data_last_read;
        double time_hw_status_last_read;
        double hw_data_write_frequency;
        double hw_data_read_frequency;
        double hw_status_read_frequency;

        // enable flags

        bool read_position_enable;
        bool read_velocity_enable;
        bool read_torque_enable;
        bool read_hw_status_enable; // for temperature + voltage + hw_error

        bool write_position_enable;
        bool write_velocity_enable;
        bool write_torque_enable;
        bool write_led_enable;
        bool write_torque_on_enable;
        bool write_tool_enable;
};


#endif
