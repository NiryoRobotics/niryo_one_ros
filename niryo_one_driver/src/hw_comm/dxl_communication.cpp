/*
    dxl_communication.cpp
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

#include "niryo_one_driver/dxl_communication.h"

uint16_t DxlCommunication::rad_pos_to_dxl_pos(double position_rad)
{
    return (uint16_t) ((double)DXL_MIDDLE_POSITION + (position_rad * RADIAN_TO_DEGREE * (double)DXL_TOTAL_RANGE_POSITION) / (double) DXL_TOTAL_ANGLE );
}

double DxlCommunication::dxl_pos_to_rad_pos(uint16_t position_dxl)
{
    return (double) ((((double)position_dxl - DXL_MIDDLE_POSITION) * (double)DXL_TOTAL_ANGLE) / (RADIAN_TO_DEGREE * (double)DXL_TOTAL_RANGE_POSITION));
}

DxlCommunication::DxlCommunication()
{
    // get params from rosparams
    ros::param::get("~dxl_uart_device_name", device_name);
    ros::param::get("~dxl_baudrate", uart_baudrate);

    ros::param::get("~dxl_hardware_control_loop_frequency", hw_control_loop_frequency);
    ros::param::get("~dxl_hw_write_frequency", hw_data_write_frequency);
    ros::param::get("~dxl_hw_data_read_frequency", hw_data_read_frequency);
    ros::param::get("~dxl_hw_status_read_frequency", hw_status_read_frequency);
    
    ROS_INFO("Start Dxl communication (%lf Hz)", hw_control_loop_frequency);
    ROS_INFO("Writing data on Dxl at %lf Hz", hw_data_write_frequency);
    ROS_INFO("Reading data from Dxl at %lf Hz", hw_data_read_frequency);
    ROS_INFO("Reading hardware error status from Dxl at %lf Hz", hw_status_read_frequency);

    resetHardwareControlLoopRates();

    dxlPortHandler = dynamixel::PortHandler::getPortHandler(device_name.c_str());
    dxlPacketHandler = dynamixel::PacketHandler::getPacketHandler(DXL_BUS_PROTOCOL_VERSION);

    xl320.reset(new XL320Driver(dxlPortHandler, dxlPacketHandler));

    is_dxl_connection_ok = false;
    debug_error_message = "No connection with Dynamixel motors has been made yet";

    // get required and authorized motors ids
    std::vector<int> required_dxl_ids;
    std::vector<int> allowed_dxl;
    ros::param::get("/niryo_one/motors/dxl_required_motors", required_dxl_ids);
    ros::param::get("/niryo_one/motors/dxl_authorized_motors", allowed_dxl);
    
    niryo_one_motors_ids.insert(niryo_one_motors_ids.end(), required_dxl_ids.begin(), required_dxl_ids.end());
    allowed_motors_ids.insert(allowed_motors_ids.end(), required_dxl_ids.begin(), required_dxl_ids.end());
    allowed_motors_ids.insert(allowed_motors_ids.end(), allowed_dxl.begin(), allowed_dxl.end());

    // Create motors
    m5_1 = DxlMotorState("Servo Axis 5_1", DXL_MOTOR_5_1_ID, DXL_MIDDLE_POSITION);
    m5_2 = DxlMotorState("Servo Axis 5_2", DXL_MOTOR_5_2_ID, DXL_MIDDLE_POSITION);
    m6 = DxlMotorState("Servo Axis 6", DXL_MOTOR_6_ID, DXL_MIDDLE_POSITION);

    for (uint8_t i = 0 ; i < required_dxl_ids.size() ; i++) {
        if      (required_dxl_ids.at(i) == m5_1.getId()) { m5_1.enable(); }
        else if (required_dxl_ids.at(i) == m5_2.getId()) { m5_2.enable(); }
        else if (required_dxl_ids.at(i) == m6.getId()) { m6.enable(); }
        else {
            debug_error_message = "Incorrect configuration : Wrong ID )" + std::to_string(required_dxl_ids.at(i)) 
                + ") given in Ros Param /niryo_one_motors/dxl_required_motors. You need to fix this !";
            ROS_ERROR("%s", debug_error_message.c_str());
            return;
        }
    }
    if (required_dxl_ids.size() == 0) {
        debug_error_message = "Incorrect configuration : Ros Param /niryo_one_motors/dxl_required_motors "
        "should contain a list with at least one motor. You need to fix this !";
        ROS_ERROR("%s", debug_error_message.c_str());
        return;
    }

    // Fill motors array 
    motors.push_back(&m5_1);
    motors.push_back(&m5_2);
    motors.push_back(&m6);

    tool = DxlMotorState("No tool connected", 0, DXL_MIDDLE_POSITION);
    is_tool_connected = false;
    
    torque_on = 0;
    
    // for hardware control loop
    hw_is_busy = false;
    hw_limited_mode = true;
    
    read_position_enable = true;
    read_velocity_enable = false; // not useful for now
    read_torque_enable = true;
    read_hw_status_enable = true;

    // change those values according to the current loaded controller (position, velocity, or torque control)
    setControlMode(DXL_CONTROL_MODE_POSITION);
    write_led_enable = true;
    write_torque_on_enable = true;
    write_tool_enable = false;
}

bool DxlCommunication::isOnLimitedMode()
{
    return hw_limited_mode;
}

void DxlCommunication::resetHardwareControlLoopRates()
{
    double now = ros::Time::now().toSec();
    time_hw_data_last_write = now;
    time_hw_data_last_read = now;
    time_hw_status_last_read = now;
}

int DxlCommunication::init()
{
    ROS_INFO("Dxl : set port name (%s), baudrate(%d)", device_name.c_str(), uart_baudrate);
    // setup half-duplex direction GPIO
    // see schema http://support.robotis.com/en/product/actuator/dynamixel_x/xl-series_main.htm
    if (!dxlPortHandler->setupGpio()) {
        ROS_ERROR("Failed to setup direction GPIO pin for Dynamixel half-duplex serial");
        return DXL_FAIL_SETUP_GPIO;
    }

    // Open port
    if (!dxlPortHandler->openPort()) {
        ROS_ERROR("Failed to open Uart port for Dynamixel bus");
        return DXL_FAIL_OPEN_PORT;
    }

    // Set baudrate
    if (!dxlPortHandler->setBaudRate(uart_baudrate)) {
        ROS_ERROR("Failed to set baudrate for Dynamixel bus");
        return DXL_FAIL_PORT_SET_BAUDRATE;
    }

    ros::Duration(0.1).sleep();
    return COMM_SUCCESS;
}

void DxlCommunication::startHardwareControlLoop(bool limited_mode)
{
    ROS_INFO("DXL : Start hardware control loop");
    hw_fail_counter_read = 0;
    write_led_enable = true;
    write_torque_on_enable = true;
    resetHardwareControlLoopRates();
    hw_control_loop_keep_alive = true;
        
    // depends on limited_mode flag
    write_position_enable = !limited_mode;
    hw_limited_mode = limited_mode;

    if (!hardware_control_loop_thread) {
        ROS_ERROR("START ctrl loop thread dxl");
        hardware_control_loop_thread.reset(new std::thread(boost::bind(&DxlCommunication::hardwareControlLoop, this)));
    }
}

void DxlCommunication::stopHardwareControlLoop()
{
    for (int i = 0; i < motors.size(); i++) {
        motors.at(i)->resetState();
    }
    tool.resetState();
    hw_control_loop_keep_alive = false;
}

void DxlCommunication::hardwareControlRead()
{
    std::vector<uint8_t> id_list;
    for (int i = 0; i < motors.size(); i++) {
        if (motors.at(i)->isEnabled()) {
            id_list.push_back(motors.at(i)->getId());
        }
    }

    if (is_tool_connected) {
        id_list.push_back(tool.getId());
    }

    if (id_list.size() == 0) {
        return; // no motor, nothing to read
    }
    
    // read data
    if (ros::Time::now().toSec() - time_hw_data_last_read > 1.0/hw_data_read_frequency)
    {
        time_hw_data_last_read += 1.0/hw_data_read_frequency;
        
        // read position
        if (read_position_enable) {
            std::vector<uint32_t> position_list;
            int read_position_result = xl320->syncReadPosition(id_list, position_list);
            if (read_position_result == COMM_SUCCESS && id_list.size() == position_list.size()) {
                hw_fail_counter_read = 0;
                int index_counter = 0;
                for (int i = 0; i < motors.size(); i++) {
                    if (motors.at(i)->isEnabled()) {
                        motors.at(i)->setPositionState(position_list.at(index_counter));
                        index_counter++;
                    }
                }
                if (is_tool_connected) {
                    tool.setPositionState(position_list.at(index_counter));
                }
            }
            else {
                hw_fail_counter_read++;
                //ROS_WARN("Fail to read position");
            }
        }

        // read velocity
        if (read_velocity_enable) {
            std::vector<uint32_t> velocity_list;
            int read_velocity_result = xl320->syncReadVelocity(id_list, velocity_list);
            if (read_velocity_result == COMM_SUCCESS && id_list.size() == velocity_list.size()) {
                hw_fail_counter_read = 0;
                int index_counter = 0;
                for (int i = 0; i < motors.size(); i++) {
                    if (motors.at(i)->isEnabled()) {
                        motors.at(i)->setVelocityState(velocity_list.at(index_counter));
                        index_counter++;
                    }
                }
                if (is_tool_connected) {
                    tool.setVelocityState(velocity_list.at(index_counter));
                }
            }
            else {
                hw_fail_counter_read++;
                //ROS_WARN("Fail to read velocity");
            }
        }

        // read load
        if (read_torque_enable) {
            std::vector<uint32_t> torque_list;
            int read_torque_result = xl320->syncReadLoad(id_list, torque_list);
            if (read_torque_result == COMM_SUCCESS && id_list.size() == torque_list.size()) {
                hw_fail_counter_read = 0;
                int index_counter = 0;
                for (int i = 0; i < motors.size(); i++) {
                    if (motors.at(i)->isEnabled()) {
                        motors.at(i)->setTorqueState(torque_list.at(index_counter));
                        index_counter++;
                    }
                }
                if (is_tool_connected) {
                    tool.setTorqueState(torque_list.at(index_counter));
                }
            }
            else {
                hw_fail_counter_read++;
                //ROS_WARN("Fail to read torque");
            }
        }
    }

    // read hardware status
    if (read_hw_status_enable) {
        if (ros::Time::now().toSec() - time_hw_status_last_read > 1.0/hw_status_read_frequency)
        {
            time_hw_status_last_read += 1.0/hw_status_read_frequency;
            
            // read temperature
            std::vector<uint32_t> temperature_list;
            int read_temperature_result = xl320->syncReadTemperature(id_list, temperature_list);

            if (read_temperature_result == COMM_SUCCESS && id_list.size() == temperature_list.size()) {
                hw_fail_counter_read = 0;
                int index_counter = 0;
                for (int i = 0; i < motors.size(); i++) {
                    if (motors.at(i)->isEnabled()) {
                        motors.at(i)->setTemperatureState(temperature_list.at(index_counter));
                        index_counter++;
                    }
                }
                if (is_tool_connected) {
                    tool.setTemperatureState(temperature_list.at(index_counter));
                }
            }
            else {
                hw_fail_counter_read++;
                //ROS_WARN("Fail to read temperature");
            }

            // read voltage
            std::vector<uint32_t> voltage_list;
            int read_voltage_result = xl320->syncReadVoltage(id_list, voltage_list);

            if (read_voltage_result == COMM_SUCCESS && id_list.size() == voltage_list.size()) {
                hw_fail_counter_read = 0;
                int index_counter = 0;
                for (int i = 0; i < motors.size(); i++) {
                    if (motors.at(i)->isEnabled()) {
                        motors.at(i)->setVoltageState(voltage_list.at(index_counter));
                        index_counter++;
                    }
                }
                if (is_tool_connected) {
                    tool.setVoltageState(voltage_list.at(index_counter));
                }
            }
            else {
                hw_fail_counter_read++;
                //ROS_WARN("Fail to read voltage");
            }

            // read hw_error
            std::vector<uint32_t> hw_error_list;
            int read_hw_error_result = xl320->syncReadHwErrorStatus(id_list, hw_error_list);

            if (read_hw_error_result == COMM_SUCCESS && id_list.size() == hw_error_list.size()) {
                hw_fail_counter_read = 0;
                int index_counter = 0;
                for (int i = 0; i < motors.size(); i++) {
                    if (motors.at(i)->isEnabled()) {
                        motors.at(i)->setHardwareError(hw_error_list.at(index_counter));
                        index_counter++;
                    }
                }
                if (is_tool_connected) {
                    tool.setHardwareError(hw_error_list.at(index_counter));
                }
            }
            else {
                hw_fail_counter_read++;
                //ROS_WARN("Fail to read hardware error status");
            }
        }
    }
   
    if (hw_fail_counter_read > 25) {
        ROS_ERROR("Dxl connection problem");
        hw_fail_counter_read = 0;
        is_dxl_connection_ok = false;
        debug_error_message = "Connection problem with Dynamixel Bus.";
    }
}

void DxlCommunication::hardwareControlWrite()
{
    std::vector<uint8_t> id_list;
    for (int i = 0; i < motors.size(); i++) {
        if (motors.at(i)->isEnabled()) {
            id_list.push_back(motors.at(i)->getId());
        }
    }

    if (ros::Time::now().toSec() - time_hw_data_last_write > 1.0/hw_data_write_frequency) {
    
        time_hw_data_last_write += 1.0/hw_data_write_frequency;

        // write torque enable
        if (write_torque_on_enable)
        {
            std::vector<uint32_t> torque_enable_list;
            for (int i = 0; i < motors.size(); i++) {
                if (motors.at(i)->isEnabled()) {
                    torque_enable_list.push_back(torque_on);
                }
            }
           
            if (is_tool_connected) {
                id_list.push_back(tool.getId());
                torque_enable_list.push_back(torque_on);
            }

            int write_torque_enable_result = xl320->syncWriteTorqueEnable(id_list, torque_enable_list);
            if (write_torque_enable_result != COMM_SUCCESS) { ROS_WARN("Fail to write torque enable"); }
            else { write_torque_on_enable = false; } // disable writing torque ON/OFF after success

            if (is_tool_connected) {
                id_list.pop_back();
            }
        }

        if (torque_on) {
            // write position
            if (write_position_enable) {
                std::vector<uint32_t> position_list;
                for (int i = 0; i < motors.size(); i++) {
                    if (motors.at(i)->isEnabled()) {
                        position_list.push_back(motors.at(i)->getPositionCommand());
                    }
                }

                int write_position_result = xl320->syncWritePositionGoal(id_list, position_list);
                if (write_position_result != COMM_SUCCESS) { ROS_WARN("Fail to write position"); }
            }

            // write velocity
            if (write_velocity_enable) {
                std::vector<uint32_t> velocity_list;
                for (int i = 0; i < motors.size(); i++) {
                    if (motors.at(i)->isEnabled()) {
                        velocity_list.push_back(motors.at(i)->getVelocityCommand());
                    }
                }
                int write_velocity_result = xl320->syncWriteVelocityGoal(id_list, velocity_list);
                if (write_velocity_result != COMM_SUCCESS) { ROS_WARN("Fail to write velocity"); }
            }

            // write torque
            if (write_torque_enable) {
                std::vector<uint32_t> torque_list;
                for (int i = 0; i < motors.size(); i++) {
                    if (motors.at(i)->isEnabled()) {
                        torque_list.push_back(motors.at(i)->getTorqueCommand());
                    }
                }
                int write_torque_result = xl320->syncWriteTorqueGoal(id_list, torque_list);
                if (write_torque_result != COMM_SUCCESS) { ROS_WARN("Fail to write torque"); }
            }

            // write_tool separately - send position, velocity and torque together
            if (write_tool_enable && is_tool_connected) {
                ros::Duration(0.005).sleep();
                int write_tool_velocity_result = xl320->setGoalVelocity(tool.getId(), tool.getVelocityCommand());
                ros::Duration(0.005).sleep();
                int write_tool_position_result = xl320->setGoalPosition(tool.getId(), tool.getPositionCommand());
                ros::Duration(0.005).sleep();
                int write_tool_torque_result = xl320->setGoalTorque(tool.getId(), tool.getTorqueCommand());
                
                if (write_tool_velocity_result != COMM_SUCCESS || 
                        write_tool_position_result != COMM_SUCCESS ||
                        write_tool_torque_result != COMM_SUCCESS) {
                    ROS_WARN("Failed to write on tool");
                }
                else {
                    write_tool_enable = false; // disable writing on tool after success
                }
            }
        }

        if (write_led_enable) {
            std::vector<uint32_t> led_list;
            for (int i = 0; i < motors.size(); i++) {
                if (motors.at(i)->isEnabled()) {
                    led_list.push_back(motors.at(i)->getLedCommand());
                }
            }

            if (is_tool_connected) {
                id_list.push_back(tool.getId());
                led_list.push_back(tool.getLedCommand());
            }

            int write_led_result = xl320->syncWriteLed(id_list, led_list);
            if (write_led_result != COMM_SUCCESS) { ROS_WARN("Fail to write led"); }
            else { write_led_enable = false; } // disable writing LED after success

            if (is_tool_connected) {
                id_list.pop_back();
            }
        }
    }
}

void DxlCommunication::hardwareControlLoop()
{
    ros::Rate hw_control_loop_rate = ros::Rate(hw_control_loop_frequency); 

    while (ros::ok()) {
        if (!hw_is_busy && hw_control_loop_keep_alive) {
            hw_is_busy = true;
            
            hardwareControlRead();
            hardwareControlWrite();

            hw_is_busy = false;
            hw_control_loop_rate.sleep();
        }
        else {
            ros::Duration(TIME_TO_WAIT_IF_BUSY).sleep(); 
            resetHardwareControlLoopRates();
           // ROS_INFO("HW control loop, wait because is busy");
        }
    }
}

void DxlCommunication::setControlMode(int control_mode)
{
    write_position_enable = (control_mode == DXL_CONTROL_MODE_POSITION);
    write_velocity_enable = (control_mode == DXL_CONTROL_MODE_VELOCITY); // not implemented yet
    write_torque_enable = (control_mode == DXL_CONTROL_MODE_TORQUE);     // not implemented yet
}

void DxlCommunication::setGoalPosition(double axis_5_pos, double axis_6_pos) 
{
    // m5_1 and m5_2 have symetric position (rad 0.0 -> position 511 for both)
    m5_1.setPositionCommand(rad_pos_to_dxl_pos(axis_5_pos));
    m5_2.setPositionCommand(DXL_MIDDLE_POSITION * 2 - m5_1.getPositionCommand());
    m6.setPositionCommand(rad_pos_to_dxl_pos(axis_6_pos));
    
    // if motor disabled, pos_state = pos_cmd (echo position)
    for (int i = 0 ; i < motors.size(); i++) {
        if (!motors.at(i)->isEnabled()) {
            motors.at(i)->setPositionState(motors.at(i)->getPositionCommand());
        }
    }
}

void DxlCommunication::getCurrentPosition(double *axis_5_pos, double *axis_6_pos)
{
    if (m5_1.isEnabled()) {
        *axis_5_pos = dxl_pos_to_rad_pos(m5_1.getPositionState());
    }
    else { // in case motor 5_1 is disabled, take motor 5_2 (symetric) position for axis 5
        *axis_5_pos = dxl_pos_to_rad_pos(DXL_MIDDLE_POSITION * 2 - m5_2.getPositionState());
    }
    *axis_6_pos = dxl_pos_to_rad_pos(m6.getPositionState());
}

void DxlCommunication::getHardwareStatus(bool *is_connection_ok, std::string &error_message, 
        int *calibration_needed, bool *calibration_in_progress,
        std::vector<std::string> &motor_names, std::vector<std::string> &motor_types,
        std::vector<int32_t> &temperatures, std::vector<double> &voltages,
        std::vector<int32_t> &hw_errors)
{
    *(is_connection_ok) = is_dxl_connection_ok;
    *(calibration_needed) = 0; // no need for calibrating dxl motors
    *(calibration_in_progress) = false; // no need for calibrating dxl motors 
    error_message = debug_error_message;

    motor_names.clear();
    motor_types.clear();
    temperatures.clear();
    voltages.clear();
    hw_errors.clear();

    for (int i = 0; i < motors.size(); i++) {
        motor_names.push_back(motors.at(i)->getName());
        motor_types.push_back("dxl_xl_320");
        temperatures.push_back(motors.at(i)->getTemperatureState());
        voltages.push_back((double)motors.at(i)->getVoltageState() / 10.0);
        hw_errors.push_back(motors.at(i)->getHardwareErrorState());
    }
    
    motor_names.push_back(tool.getName());
    motor_types.push_back("dxl_xl_320");
    temperatures.push_back(tool.getTemperatureState());
    voltages.push_back((double)tool.getVoltageState() / 10.0);
    hw_errors.push_back(tool.getHardwareErrorState());
}

bool DxlCommunication::isConnectionOk()
{
    return is_dxl_connection_ok;
}

void DxlCommunication::setTorqueOn(bool on)
{
    if (hw_limited_mode) { // only allow to activate torque if limited_mode is OFF
        torque_on = false;
        write_torque_on_enable = true;
    }
    else {
        torque_on = on;
        write_torque_on_enable = true;
    }
}

void DxlCommunication::setLeds(std::vector<int> &leds)
{
    if (leds.size() < motors.size() + 1) {
        ROS_WARN("Led array must contain %d values", motors.size() + 1);
        return;
    }

    int index_counter = 0;

    for (int i = 0; i < motors.size(); i++) {
        if (motors.at(i)->isEnabled()) {
            if (leds.at(index_counter) >= 0 && leds.at(index_counter) <= 7) {
                motors.at(i)->setLedCommand(leds.at(index_counter));
            }
            index_counter++;
        }
    }

    if (leds.at(index_counter) >= 0 && leds.at(index_counter) <= 7) {
        tool.setLedCommand(leds.at(index_counter));
    }

    write_led_enable = true;
}

void DxlCommunication::setTool(uint8_t id, std::string name)
{
    is_tool_connected = (id > 0);
    tool.setId(id);  // id "0" means no tool
    tool.setName(name);
    tool.resetState();
    tool.resetCommand();

    ROS_INFO("Set tool with id : %d", tool.getId());
}

int DxlCommunication::pingAndSetTool(uint8_t id, std::string name)
{
    if (id == 0) { // detach tool
        setTool(0, "No Dxl Tool");
        return TOOL_STATE_PING_OK;
    }

    while (hw_is_busy) { 
        ros::Duration(TIME_TO_WAIT_IF_BUSY).sleep();    
    }
    hw_is_busy = true;

    int retries = 3;
    int ping_result = COMM_RX_FAIL;

    while (retries > 0) {
        ping_result = xl320->ping(id);
        if (ping_result == COMM_SUCCESS) {
            retries = 0;
        }
        else {
            retries--;
        }
    }

    ROS_INFO("Ping Tool : ping result for id (%d) : %d", id, ping_result);

    hw_is_busy = false;
    
    if (ping_result != COMM_SUCCESS) {
        setTool(0, "No Dxl Tool"); // no tool detected
        return TOOL_STATE_PING_ERROR;
    }

    setTool(id, name); // if ping successful, we update current attached tool
    return TOOL_STATE_PING_OK;
}

/*
 * This method should be called in a different thread than control loop
 */
int DxlCommunication::openGripper(uint8_t id, uint16_t open_position, uint16_t open_speed, uint16_t open_hold_torque)
{
    // check gripper id, in case no ping has been done before, or wrong id given
    if (id != tool.getId()) {
        return TOOL_STATE_WRONG_ID;  
    }

    // set gripper pos, vel and torque
    tool.setVelocityCommand(open_speed);
    tool.setPositionCommand(open_position);
    tool.setTorqueCommand(1023);
    write_tool_enable = true;

    // calculate open duration
    int dxl_speed = open_speed * DXL_STEPS_FOR_1_SPEED; // position . sec-1
    int dxl_steps_to_do = abs(open_position - tool.getPositionState()); // position
    double seconds_to_wait = (double) dxl_steps_to_do / (double) dxl_speed; // sec
   
    ros::Duration(seconds_to_wait + 0.25).sleep();
    
    // set hold torque
    tool.setTorqueCommand(open_hold_torque);
    write_tool_enable = true;

    return GRIPPER_STATE_OPEN;
}

/*
 * Close position must be lower than open position (from mechanical design)
 * This method should be called in a different thread than control loop
 */
int DxlCommunication::closeGripper(uint8_t id, uint16_t close_position, uint16_t close_speed, uint16_t close_hold_torque, uint16_t close_max_torque)
{
    // check gripper id, in case no ping has been done before, or wrong id given
    if (id != tool.getId()) {
        return TOOL_STATE_WRONG_ID;  
    }
   
    int position_command = (close_position < 50) ? 0 : close_position - 50;

    // set gripper pos, vel and torque
    tool.setVelocityCommand(close_speed);
    tool.setPositionCommand(position_command);
    tool.setTorqueCommand(close_max_torque);
    write_tool_enable = true;

    // calculate close duration
    int dxl_speed = close_speed * DXL_STEPS_FOR_1_SPEED; // position . sec-1
    int dxl_steps_to_do = abs(close_position - tool.getPositionState()); // position
    double seconds_to_wait = (double) dxl_steps_to_do / (double) dxl_speed; // sec

    ros::Duration(seconds_to_wait + 0.25).sleep();
    
    // set hold torque and position
    tool.setTorqueCommand(close_hold_torque);
    tool.setPositionCommand(close_position);
    write_tool_enable = true;

    return GRIPPER_STATE_CLOSE;
}

/*
 * This method should be called in a different thread than control loop
 */
int DxlCommunication::pullAirVacuumPump(uint8_t id, uint16_t pull_air_position, uint16_t pull_air_hold_torque)
{
    // check gripper id, in case no ping has been done before, or wrong id given
    if (id != tool.getId()) {
        return TOOL_STATE_WRONG_ID;  
    }
   
    int pull_air_velocity = 1023;

    // set vacuum pump pos, vel and torque
    tool.setVelocityCommand(pull_air_velocity);
    tool.setPositionCommand(pull_air_position);
    tool.setTorqueCommand(1023);
    write_tool_enable = true;

    // calculate pull air duration
    int dxl_speed = pull_air_velocity * DXL_STEPS_FOR_1_SPEED; // position . sec-1
    int dxl_steps_to_do = abs(pull_air_position - tool.getPositionState()); // position
    double seconds_to_wait = (double) dxl_steps_to_do / (double) dxl_speed; // sec
    
    ros::Duration(seconds_to_wait + 0.25).sleep();
    
    // set hold torque
    tool.setTorqueCommand(pull_air_hold_torque);
    write_tool_enable = true;

    return VACUUM_PUMP_STATE_PULLED;
}

/*
 * This method should be called in a different thread than control loop
 */
int DxlCommunication::pushAirVacuumPump(uint8_t id, uint16_t push_air_position)
{
    // check gripper id, in case no ping has been done before, or wrong id given
    if (id != tool.getId()) {
        return TOOL_STATE_WRONG_ID;  
    }

    int push_air_velocity = 1023;

    // set vacuum pump pos, vel and torque
    tool.setVelocityCommand(push_air_velocity);
    tool.setPositionCommand(push_air_position);
    tool.setTorqueCommand(1023);
    write_tool_enable = true;

    // calculate push air duration
    int dxl_speed = push_air_velocity * DXL_STEPS_FOR_1_SPEED; // position . sec-1
    int dxl_steps_to_do = abs(push_air_position - tool.getPositionState()); // position
    double seconds_to_wait = (double) dxl_steps_to_do / (double) dxl_speed; // sec
    
    ros::Duration(seconds_to_wait + 0.25).sleep();

    // set torque to 0
    tool.setTorqueCommand(0);
    write_tool_enable = true;

    return VACUUM_PUMP_STATE_PUSHED;
}
        
int DxlCommunication::scanAndCheck() 
{
    int counter = 0;

    while (hw_is_busy && counter < 100) { 
        ros::Duration(TIME_TO_WAIT_IF_BUSY).sleep();    
        counter++;
    }
    
    if (counter == 100) {
        debug_error_message = "Failed to scan motors, Dynamixel bus is too busy. Will retry...";
        ROS_WARN("Failed to scan motors, dxl bus is too busy (counter max : %d)", counter);
        return COMM_PORT_BUSY;
    }
   
    hw_is_busy = true;

    // 1. Get all ids from dxl bus
    std::vector<uint8_t> id_list;
    int result = xl320->scan(id_list);
    hw_is_busy = false;
    
    if (result != COMM_SUCCESS) {
        debug_error_message = "Failed to scan Dynamixel motors. Make sure that motors are correctly connected.";
        ROS_WARN("Broadcast ping failed , result : %d", result);
        return result;
    }

    // 2. Check that ids correspond to niryo_one motors id list
    std::vector<uint8_t>::iterator it;

    for (it = niryo_one_motors_ids.begin() ; it < niryo_one_motors_ids.end() ; it++) {
        if (std::find(id_list.begin(), id_list.end(), *it) == id_list.end()) {
            debug_error_message = "Missing Dynamixel motor(s) on the robot. Make sure that all motors are correctly connected.";
            ROS_ERROR("Missing Dynamixel : %d", *it);
            return DXL_SCAN_MISSING_MOTOR;
        }
    }

    if (is_tool_connected) {
        if (std::find(id_list.begin(), id_list.end(), tool.getId()) == id_list.end()) {
            debug_error_message = "Missing tool. Make sure that all motors are correctly connected.";
            ROS_ERROR("Missing Dynamixel : %d", tool.getId());
            return DXL_SCAN_MISSING_MOTOR;
        }
    }

    // 3. Check that there is no unwanted motor
    for (it = id_list.begin() ; it < id_list.end() ; it++) {
        if (std::find(allowed_motors_ids.begin(), allowed_motors_ids.end(), *it) == allowed_motors_ids.end()) {
            debug_error_message = "Unallowed Dynamixel motor(s) on the robot. Make sure that all motors id are correct. "
                "You may need to reconfigure some Dynamixel motors.";
            ROS_ERROR("Unallowed Dynamixel : %d", *it);
            return DXL_SCAN_UNALLOWED_MOTOR;
        }
    }
   
    is_dxl_connection_ok = true;
    debug_error_message = "";
    return DXL_SCAN_OK;
}
