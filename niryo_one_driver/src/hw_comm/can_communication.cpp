/*
    can_communication.cpp
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

#include "niryo_one_driver/can_communication.h"

int32_t CanCommunication::rad_pos_to_steps(double position_rad, double gear_ratio, double direction)
{
    return (int32_t) ((200.0 * 8.0 * gear_ratio * position_rad * RADIAN_TO_DEGREE / 360.0) * direction);
}

double CanCommunication::steps_to_rad_pos(int32_t steps, double gear_ratio, double direction)
{
    return (double) ((double)steps * 360.0 / (200.0 * 8.0 * gear_ratio * RADIAN_TO_DEGREE)) * direction ;
}

CanCommunication::CanCommunication()
{
    ros::param::get("~spi_channel", spi_channel);
    ros::param::get("~spi_baudrate", spi_baudrate);
    ros::param::get("~gpio_can_interrupt", gpio_can_interrupt);

    // set frequencies for hw control loop
    ros::param::get("~can_hardware_control_loop_frequency", hw_control_loop_frequency);
    ros::param::get("~can_hw_write_frequency", hw_write_frequency);
    ros::param::get("~can_hw_check_connection_frequency", hw_check_connection_frequency);

    ROS_INFO("Start CAN communication (%lf Hz)", hw_control_loop_frequency);
    ROS_INFO("Writing data on CAN at %lf Hz", hw_write_frequency);
    ROS_INFO("Checking CAN connection at %lf Hz", hw_check_connection_frequency);
    
    resetHardwareControlLoopRates();

    // start can driver
    can.reset(new NiryoCanDriver(spi_channel, spi_baudrate, gpio_can_interrupt));

    is_can_connection_ok = false;
    debug_error_message = "No connection with CAN motors has been made yet";
    
    // get connected motors from rosparams
    ros::param::get("/niryo_one/motors/can_required_motors", required_steppers_ids);
    
    double gear_ratio_1, gear_ratio_2, gear_ratio_3, gear_ratio_4;
    ros::param::get("~stepper_1_gear_ratio", gear_ratio_1);
    ros::param::get("~stepper_2_gear_ratio", gear_ratio_2);
    ros::param::get("~stepper_3_gear_ratio", gear_ratio_3);
    ros::param::get("~stepper_4_gear_ratio", gear_ratio_4);
    ROS_INFO("Gear ratios : (1 : %lf, 2 : %lf, 3 : %lf, 4 : %lf)", gear_ratio_1, gear_ratio_2, gear_ratio_3, gear_ratio_4);

    double home_position_1, home_position_2, home_position_3, home_position_4;
    ros::param::get("~stepper_1_home_position", home_position_1);
    ros::param::get("~stepper_2_home_position", home_position_2);
    ros::param::get("~stepper_3_home_position", home_position_3);
    ros::param::get("~stepper_4_home_position", home_position_4);
    ROS_INFO("Home positions : (1 : %lf, 2 : %lf, 3 : %lf, 4 : %lf)", home_position_1, home_position_2, home_position_3, home_position_4);

    double offset_position_1, offset_position_2, offset_position_3, offset_position_4;
    ros::param::get("~stepper_1_offset_position", offset_position_1);
    ros::param::get("~stepper_2_offset_position", offset_position_2);
    ros::param::get("~stepper_3_offset_position", offset_position_3);
    ros::param::get("~stepper_4_offset_position", offset_position_4);
    ROS_INFO("Angle offsets : (1 : %lf, 2 : %lf, 3 : %lf, 4 : %lf)", offset_position_1, offset_position_2, offset_position_3, offset_position_4);

    double direction_1, direction_2, direction_3, direction_4;
    ros::param::get("~stepper_1_direction", direction_1);
    ros::param::get("~stepper_2_direction", direction_2); 
    ros::param::get("~stepper_3_direction", direction_3);
    ros::param::get("~stepper_4_direction", direction_4);

    int max_effort_1, max_effort_2, max_effort_3, max_effort_4;
    ros::param::get("~stepper_1_max_effort", max_effort_1);
    ros::param::get("~stepper_2_max_effort", max_effort_2);
    ros::param::get("~stepper_3_max_effort", max_effort_3);
    ros::param::get("~stepper_4_max_effort", max_effort_4);

    // Create motors with previous params
    m1 = StepperMotorState("Stepper Axis 1", CAN_MOTOR_1_ID, gear_ratio_1, direction_1, 
            rad_pos_to_steps(home_position_1, gear_ratio_1, direction_1),            // home position
            rad_pos_to_steps(offset_position_1, gear_ratio_1, direction_1),          // offset position
            8, max_effort_1);
    m2 = StepperMotorState("Stepper Axis 2", CAN_MOTOR_2_ID, gear_ratio_2, direction_2,
            rad_pos_to_steps(home_position_2, gear_ratio_2, direction_2),
            rad_pos_to_steps(offset_position_2, gear_ratio_2, direction_2),
            8, max_effort_2);
    m3 = StepperMotorState("Stepper Axis 3", CAN_MOTOR_3_ID, gear_ratio_3, direction_3, 
            rad_pos_to_steps(home_position_3, gear_ratio_3, direction_3),
            rad_pos_to_steps(offset_position_3, gear_ratio_3, direction_3),
            8, max_effort_3);
    m4 = StepperMotorState("Stepper Axis 4", CAN_MOTOR_4_ID, gear_ratio_4, direction_4,
            rad_pos_to_steps(home_position_4, gear_ratio_4, direction_4),
            rad_pos_to_steps(offset_position_4, gear_ratio_4, direction_4),
            8, max_effort_4);

    for (uint8_t i = 0 ; i < required_steppers_ids.size() ; ++i) {
        if      (required_steppers_ids.at(i) == m1.getId()) { m1.enable(); }
        else if (required_steppers_ids.at(i) == m2.getId()) { m2.enable(); }
        else if (required_steppers_ids.at(i) == m3.getId()) { m3.enable(); }
        else if (required_steppers_ids.at(i) == m4.getId()) { m4.enable(); }
        else {
            debug_error_message = "Incorrect configuration : Wrong ID given in Ros Param /niryo_one_motors/can_required_motors. " 
            "You need to fix this !";
            ROS_ERROR("%s", debug_error_message.c_str());
            return;
        }
    }
    if (required_steppers_ids.size() == 0) {
        debug_error_message = "Incorrect configuration : Ros Param /niryo_one_motors/can_required_motors "
        "should contain a list with at least one motor. You need to fix this !";
        ROS_ERROR("%s", debug_error_message.c_str());
        return;
    }

    ROS_INFO("%d motors should be connected to CAN bus", (int) required_steppers_ids.size());

    // fill motors array (to avoid redundant code later)
    motors.push_back(&m1);
    motors.push_back(&m2);
    motors.push_back(&m3);
    motors.push_back(&m4);

    // set hw control init state
    torque_on = 0;

    hw_is_busy = false;
    hw_limited_mode = true;

    write_position_enable = true;
    write_torque_enable = false;
    write_torque_on_enable = true;

    write_synchronize_enable = false;
    write_micro_steps_enable =  true;
    write_max_effort_enable = true;

    waiting_for_user_trigger_calibration = false;
    steppers_calibration_mode = CAN_STEPPERS_CALIBRATION_MODE_AUTO; // default
    write_synchronize_begin_traj = true;
    calibration_in_progress = false;
}

int CanCommunication::init()
{
    int gpio_result = can->setupInterruptGpio();
    if (gpio_result != CAN_OK) {
        ROS_ERROR("Failed to start gpio for CAN bus"); 
        return gpio_result;
    }

    int spi_result = can->setupSpi();
    if (spi_result != CAN_OK) {
        ROS_ERROR("Failed to start spi communication for CAN bus"); 
        return spi_result;
    }

    return can->init();
}

bool CanCommunication::isOnLimitedMode() 
{
    return hw_limited_mode;
}

void CanCommunication::resetHardwareControlLoopRates()
{
    double now = ros::Time::now().toSec();
    time_hw_last_write = now;
    time_hw_last_check_connection = now;
}

void CanCommunication::startHardwareControlLoop(bool limited_mode)
{
    ROS_INFO("CAN : Start hardware control loop");
    write_torque_on_enable = true;
    write_micro_steps_enable = true;
    write_max_effort_enable = true;
    resetHardwareControlLoopRates();
        
    // depends on limited_mode flag
    write_position_enable = !limited_mode;
    write_synchronize_enable = !limited_mode;
    write_torque_on_enable = true;

    hw_limited_mode = limited_mode;
    hw_control_loop_keep_alive = true;
   
    if (!hardware_control_loop_thread) {
        ROS_ERROR("START ctrl loop thread can");
        hardware_control_loop_thread.reset(new std::thread(boost::bind(&CanCommunication::hardwareControlLoop, this)));
    }
}

void CanCommunication::stopHardwareControlLoop()
{
    ROS_INFO("CanComm : Stop hardware control loop");
    for (int i = 0; i < motors.size(); i++) {
        motors.at(i)->resetState();
    }
    hw_control_loop_keep_alive = false;
}

void CanCommunication::hardwareControlRead()
{
    if (can->canReadData()) {
        long unsigned int rxId;
        unsigned char len;
        unsigned char rxBuf[8];
        
        can->readMsgBuf(&rxId, &len, rxBuf);
        
        // 0. This functionality will come later, to allow user to plug other CAN devices to RPI
        // Developement to do here : check if id >= 0x20
        // Ids between 0x00 and 0x1F are reserved for Niryo One core communication
        // Those are lower ids with higher priority, to ensure connection with motors is always up.
        if (rxId >= 0x20) {
            // send frame to another place and return
        }

        // 1. Validate motor id
        int motor_id = rxId & 0x0F; // 0x11 for id 1, 0x12 for id 2, ...
        bool motor_found = false;
        for (int i = 0; i < motors.size(); i++) {
            if (motor_id == motors.at(i)->getId()) {
                motors.at(i)->setLastTimeRead(ros::Time::now().toSec());
                motor_found = true;
                break;
            }
        }
       
        if (!motor_found) {
            ROS_ERROR("Received can frame with wrong id : %d", motor_id);
            return;
        }

        // 1.1 Check buffer is not empty
        if (len < 1) {
            ROS_ERROR("Received can frame with empty data");
            return;
        }

        // 2. If id ok, check control byte and fill data
        int control_byte = rxBuf[0];
        
        if (control_byte == CAN_DATA_POSITION) {
            // check length 
            if (len != 4) {
                ROS_ERROR("Position can frame should contain 4 data bytes");
                return;
            }
            
            int32_t pos = (rxBuf[1] << 16) + (rxBuf[2] << 8) + rxBuf[3];
            if (pos & (1 << 15)) {
            	pos = -1 * ((~pos + 1) & 0xFFFF);
          	} 
           
            // fill data
            for (int i = 0; i < motors.size() ; i++) {
                if (motor_id == motors.at(i)->getId() && motors.at(i)->isEnabled()) {
                    motors.at(i)->setPositionState(pos);
                    break;
                }
            }
        }
        else if (control_byte == CAN_DATA_DIAGNOSTICS) {
            // check data length
            if (len != 4) {
                ROS_ERROR("Diagnostic can frame should contain 4 data bytes");
                return;
            }
            int mode = rxBuf[1];
            int driver_temp_raw = (rxBuf[2] << 8) + rxBuf[3];
            double a = -0.00316;
            double b = -12.924;
            double c = 2367.7;
            double v_temp = driver_temp_raw * 3.3 / 1024.0 * 1000.0;
            int driver_temp = int((-b - std::sqrt(b*b - 4*a*(c - v_temp)))/(2*a)+30);
            
            // fill data
            for (int i = 0; i < motors.size() ; i++) {
                if (motor_id == motors.at(i)->getId() && motors.at(i)->isEnabled()) {
                    motors.at(i)->setTemperatureState(driver_temp);
                    break;
                }
            }
            //ROS_INFO("Mode : %d, Temp : %d", mode, m1.getTemperatureState());
        }
        else if (control_byte == CAN_DATA_FIRMWARE_VERSION) {
            if (len != 4) {
                ROS_ERROR("Firmware version frame should contain 4 bytes");
                return;
            }
            int v_major = rxBuf[1];
            int v_minor = rxBuf[2];
            int v_patch = rxBuf[3];
            std::string version = "";
            version += std::to_string(v_major); version += "."; 
            version += std::to_string(v_minor); version += "."; 
            version += std::to_string(v_patch);

            // fill data
            for (int i = 0; i < motors.size(); i++) {
                if (motor_id == motors.at(i)->getId() && motors.at(i)->isEnabled()) {
                    motors.at(i)->setFirmwareVersion(version);
                    return;
                }
            }
        }
        else {
            ROS_ERROR("Received can frame with unknown control byte");
            return;
        }
    
        //ROS_INFO("pos1 : %d, pos2 : %d, pos3 : %d, pos4 :%d", m1.getPositionState(), m2.getPositionState(), m3.getPositionState(), m4.getPositionState());
        //ROS_INFO("pos1 : %lf", steps_to_rad_pos(m3.getPositionState(), m3.getGearRatio(), 1.0));
    }
}

/*
 * Sends a CAN frame per motor (id + control byte + data)
 */
void CanCommunication::hardwareControlWrite()
{
    if (ros::Time::now().toSec() - time_hw_last_write > 1.0/hw_write_frequency) {
        time_hw_last_write += 1.0/hw_write_frequency;

        // write torque ON/OFF
        if (write_torque_on_enable) {
            if (can->sendTorqueOnCommand(CAN_BROADCAST_ID, torque_on) != CAN_OK) {
                ROS_ERROR("Failed to send torque on");
            }
            else {
                write_torque_on_enable = false; // disable writing on success
            }
        }
        
        // write synchronize position
        if (write_synchronize_enable) {
            bool synchronize_write_success = true;
            
            for (int i = 0 ; i < motors.size(); i++) {
                if (motors.at(i)->isEnabled()) {
                    if (can->sendSynchronizePositionCommand(motors.at(i)->getId(), write_synchronize_begin_traj) != CAN_OK) {
                        synchronize_write_success = false;
                    }
                }
            }

            if (synchronize_write_success) {
                write_synchronize_enable = false; // disable writing after success
            }
            else {
                ROS_ERROR("Failed to send synchronize position command");
            }
        }

        // write position
        if (write_position_enable) {
            
            for (int i = 0 ; i < motors.size(); i++) {
                if (motors.at(i)->isEnabled()) {
                    if (can->sendPositionCommand(motors.at(i)->getId(), motors.at(i)->getPositionCommand()) != CAN_OK) {
                        //ROS_ERROR("Failed to send position");
                    }
                }
            }
        }
       
        // write micro steps
        if (write_micro_steps_enable) {
            bool micro_steps_write_success = true;
            
            for (int i = 0 ; i < motors.size(); i++) {
                if (motors.at(i)->isEnabled()) {
                    if (can->sendMicroStepsCommand(motors.at(i)->getId(), motors.at(i)->getMicroStepsCommand()) != CAN_OK) {
                        micro_steps_write_success = false;
                    }
                }
            }

            if (micro_steps_write_success) {
                write_micro_steps_enable = false; // disable writing after success
            }
            else {
                ROS_ERROR("Failed to send Micro Steps");
            }
        }

        // write max effort
        if (write_max_effort_enable) {
            bool max_effort_write_success = true;
            
            for (int i = 0; i < motors.size(); i++) {
                if (motors.at(i)->isEnabled()) {
                    if (can->sendMaxEffortCommand(motors.at(i)->getId(), motors.at(i)->getMaxEffortCommand()) != CAN_OK) {
                        max_effort_write_success = false;
                    }
                }
            }
            
            if (max_effort_write_success) {
                write_max_effort_enable = false; // disable writing on success
            }
            else {
                ROS_ERROR("Failed to send Max Effort");
            }
        }
    }
}

void CanCommunication::hardwareControlCheckConnection()
{
    if (ros::Time::now().toSec() - time_hw_last_check_connection > 1.0/hw_check_connection_frequency) {
        time_hw_last_check_connection += 1.0/hw_check_connection_frequency;
     
        if (!is_can_connection_ok) {
            return; // don't check if connection is already lost --> need to call scanAndCheck()
        }

        double time_now = ros::Time::now().toSec();
        if (hw_check_connection_frequency > 5.0) { // if we check MCP_2515 too fast it will not work
            hw_check_connection_frequency = 5.0;
        }
        double timeout_read = 1.0/hw_check_connection_frequency;
        int max_fail_counter = (int) (hw_check_connection_frequency + 0.5); // connection error will be detected after 1 sec

        for (int i = 0; i < motors.size(); i++) {
            if (motors.at(i)->isEnabled()) {
                if (time_now - motors.at(i)->getLastTimeRead() > timeout_read * (motors.at(i)->getHwFailCounter() + 1)) {
                    ROS_ERROR("CAN connection problem with motor %d, hw fail counter : %d", motors.at(i)->getId(), motors.at(i)->getHwFailCounter());
                    if (motors.at(i)->getHwFailCounter() >= max_fail_counter) {
                        is_can_connection_ok = false;
                        debug_error_message = "Connection problem with CAN bus. Motor ";
                        debug_error_message += motors.at(i)->getId();
                        debug_error_message += " is not connected";
                        return;
                    }

                    // reset MCP_2515
                    //int result = can->init(); 
                    //ROS_INFO("Can init : %d", result);
                    motors.at(i)->setHwFailCounter(motors.at(i)->getHwFailCounter() + 1);
                }
                else {
                    motors.at(i)->setHwFailCounter(0);
                }
            }
        }        
    }
}

void CanCommunication::hardwareControlLoop()
{
    ros::Rate hw_control_loop_rate = ros::Rate(hw_control_loop_frequency); 

    while (ros::ok()) {
        if (!hw_is_busy && hw_control_loop_keep_alive) {
            hw_is_busy = true;
            
            hardwareControlRead();
            hardwareControlWrite();
            hardwareControlCheckConnection();

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

void CanCommunication::synchronizeSteppers(bool begin_traj)
{
    write_synchronize_enable = true;
    write_synchronize_begin_traj = begin_traj;
}

void CanCommunication::setTorqueOn(bool on)
{
    if (!on && is_can_connection_ok && waiting_for_user_trigger_calibration && !calibration_in_progress) {
        can->sendTorqueOnCommand(CAN_BROADCAST_ID, false); // only to deactivate motors when waiting for calibration
    }
    else if (hw_limited_mode) {
        torque_on = false;
        write_torque_on_enable = true;
    }
    else {
        torque_on = on;
        write_torque_on_enable = true;
    }
}

/*
 * User input to clear the calibration flag
 * - also choose a calibration mode (manual|auto)
 */
void CanCommunication::validateMotorsCalibrationFromUserInput(int mode)
{
    waiting_for_user_trigger_calibration = false;
    if (mode == CAN_STEPPERS_CALIBRATION_MODE_MANUAL || mode == CAN_STEPPERS_CALIBRATION_MODE_AUTO) {
        steppers_calibration_mode = mode;
    }
}

/* 
 * This flag should be cleared by calling the service /niryo_one_control/calibrate_motors
 */
void CanCommunication::setCalibrationFlag(bool flag)
{
    waiting_for_user_trigger_calibration = flag;
}

/* 
 * 1. If mode is manual, it will just send an offset to all the motors, it means the user has to place the robot
 * to home position before using this mode
 * 2. If mode is automatic, it will send a calibration command to all motors, and wait until it receives a confirmation
 * from all motors (success, timeout, bad params)
 */
int CanCommunication::calibrateMotors()
{
    // this flag should be cleared by a user action to continue
    if (waiting_for_user_trigger_calibration) {
        return CAN_STEPPERS_CALIBRATION_WAITING_USER_INPUT;
    }

    ROS_INFO("START Calibrating stepper motors");
    stopHardwareControlLoop();
    ros::Duration(0.1).sleep();

    // If user wants to do a manual calibration, just send offset to current position
    if (steppers_calibration_mode == CAN_STEPPERS_CALIBRATION_MODE_MANUAL) {
        calibration_in_progress = true;
        int result = manualCalibration();
        calibration_in_progress = false;
        return result;
    }
    else if (steppers_calibration_mode == CAN_STEPPERS_CALIBRATION_MODE_AUTO) {
        calibration_in_progress = true;
        int result = autoCalibration();
        calibration_in_progress = false;
        return result;
    }
    else {
        return CAN_STEPPERS_CALIBRATION_FAIL;
    }
}

bool CanCommunication::isCalibrationInProgress()
{
    return calibration_in_progress;
}

/*
 * Set to home position instead of offset position (more intuitive for user)
 */
int CanCommunication::manualCalibration()
{
    ROS_INFO("Manual calibration : just send offset to steppers");
    for (int i = 0 ; i < motors.size() ; i++) {
        if (motors.at(i)->isEnabled()) {
            if (can->sendPositionOffsetCommand(motors.at(i)->getId(), motors.at(i)->getHomePosition()) != CAN_OK) {
                return CAN_STEPPERS_CALIBRATION_FAIL;
            }
        }
    }
    return CAN_STEPPERS_CALIBRATION_OK;
}

int CanCommunication::sendCalibrationCommandForOneMotor(StepperMotorState* motor, int delay_between_steps,
        int calibration_direction, int calibration_timeout)
{
    if (!motor->isEnabled()) {
        return CAN_STEPPERS_CALIBRATION_OK;
    }
    
    if (can->sendCalibrationCommand(motor->getId(), motor->getOffsetPosition(), delay_between_steps,
                (int)motor->getDirection() * calibration_direction, calibration_timeout) != CAN_OK) {
        ROS_ERROR("Failed to send calibration command for motor : %d", motor->getId());
        return CAN_STEPPERS_CALIBRATION_FAIL;
    }

    return CAN_OK;
}

int CanCommunication::getCalibrationResults(std::vector<StepperMotorState*> steppers, int calibration_timeout)
{
    std::vector<int> motors_ids;
    std::vector<bool> calibration_results;

    for (int i = 0 ; i < steppers.size() ; i++) {
        if (steppers.at(i)->isEnabled()) {
            motors_ids.push_back(steppers.at(i)->getId());
            calibration_results.push_back(false);
        }
    }

    double time_begin_calibration = ros::Time::now().toSec();
    double timeout = time_begin_calibration + (double)calibration_timeout;
    ROS_INFO("Waiting for motor calibration");
    //ROS_INFO("Waiting for motor %d calibration response...", motor->getId());
    
    while (ros::Time::now().toSec() < timeout) {
        ros::Duration(0.0005).sleep(); // check at 2000 Hz

        // check if success
        bool success = true;
        for (int i = 0 ; i < calibration_results.size() ; i++) {
            if (calibration_results.at(i) == false) {
                success = false;
            }
        }
        if (success) {
            return CAN_STEPPERS_CALIBRATION_OK;
        }

        if (can->canReadData()) {
            long unsigned int rxId;
            unsigned char len;
            unsigned char rxBuf[8];
            
            can->readMsgBuf(&rxId, &len, rxBuf);
            
            // 1. Get motor id
            int motor_id = rxId & 0x00F; // 0x101 for id 1, 0x102 for id 2, ...

            // 2. Check if motor id is in array
            for (int i = 0 ; i < motors_ids.size() ; i++) {
                if (motors_ids.at(i) == motor_id) {
                    if (len == 2) {
                        // 2. Check control byte
                        int control_byte = rxBuf[0];

                        if (control_byte == CAN_DATA_CALIBRATION_RESULT) { // only check this frame
                            int result = rxBuf[1];
                            
                            if (result == CAN_STEPPERS_CALIBRATION_TIMEOUT) {
                                ROS_ERROR("Motor %d had calibration timeout", motor_id);
                                return result;
                            }
                            else if (result == CAN_STEPPERS_CALIBRATION_BAD_PARAM) {
                                ROS_ERROR("Bad params given to motor %d", motor_id);
                                return result;
                            }
                            else if (result == CAN_STEPPERS_CALIBRATION_OK) {
                                ROS_INFO("Motor %d calibration OK", motor_id);
                                calibration_results.at(i) = true;
                            }
                        }
                    }
                }
            }
        }
    }
    return CAN_STEPPERS_CALIBRATION_TIMEOUT; 
}

/*
 * To use only during calibration phase, or for debug purposes
 * - Move motor from whatever current position to current_steps + steps
 */
int CanCommunication::relativeMoveMotor(StepperMotorState* motor, int steps, int delay, bool wait)
{
    if (!motor->isEnabled()) {
        return CAN_OK;
    }

    if (can->sendTorqueOnCommand(motor->getId(), true) != CAN_OK) {
        ROS_ERROR("Failed to send torque ON to motor %d", motor->getId());
        return CAN_FAIL;
    }

    if (can->sendRelativeMoveCommand(motor->getId(), steps, delay) != CAN_OK) {
        ROS_ERROR("Relative Move motor failed for motor %d", motor->getId());
        return CAN_FAIL;
    }
    if (wait) {
        ros::Duration(steps*delay/1000000 + 0.5).sleep(); // wait for 0.5 sec more to finish
    }

    return CAN_OK;
}

int CanCommunication::autoCalibration()
{
    int calibration_timeout = 30; // seconds
    int result;

    // 0. Torque ON for motor 2
    if (can->sendTorqueOnCommand(m2.getId(), true) != CAN_OK) {
        return CAN_STEPPERS_CALIBRATION_FAIL;
    }

    // 1. Move axis 3 up
    if (relativeMoveMotor(&m3, rad_pos_to_steps(0.5, m3.getGearRatio(), m3.getDirection()), 1500, true) != CAN_OK) {
        return CAN_STEPPERS_CALIBRATION_FAIL; 
    }

    // 2. Send calibration cmd 1 + 2 + 4
    if (sendCalibrationCommandForOneMotor(&m1, 800, 1, calibration_timeout) != CAN_OK) {
        return CAN_STEPPERS_CALIBRATION_FAIL;
    }

    if (sendCalibrationCommandForOneMotor(&m2, 1100, 1, calibration_timeout) != CAN_OK) {
        return CAN_STEPPERS_CALIBRATION_FAIL;
    }

    if (sendCalibrationCommandForOneMotor(&m4, 800, 1, calibration_timeout) != CAN_OK) {
        return CAN_STEPPERS_CALIBRATION_FAIL;
    }

    // 2.1 Wait calibration result
    std::vector<StepperMotorState*> steppers = { &m1, &m2, &m4 };
    if (getCalibrationResults(steppers, calibration_timeout) != CAN_STEPPERS_CALIBRATION_OK) {
        return CAN_STEPPERS_CALIBRATION_FAIL;
    }

    // 3. Move motor 1,2,4 to 0.0
    int delay_micros = 2000;
    if (relativeMoveMotor(&m1, -m1.getOffsetPosition(), 1500, false) != CAN_OK) {
        return CAN_STEPPERS_CALIBRATION_FAIL;
    }
    if (relativeMoveMotor(&m2, -m2.getOffsetPosition(), 3000, false) != CAN_OK) {
        return CAN_STEPPERS_CALIBRATION_FAIL;
    }
    if (relativeMoveMotor(&m4, -m4.getOffsetPosition(), 1500, false) != CAN_OK) {
        return CAN_STEPPERS_CALIBRATION_FAIL;
    }
    
    // 3.1 Wait for motors to finish moving (at least m4, so we can start m3 calibration)
    ros::Duration(abs(m4.getOffsetPosition()) * 1500 / 1000000).sleep();
    
    // 5. Send calibration cmd m3
    if (sendCalibrationCommandForOneMotor(&m3, 1300, -1, calibration_timeout) != CAN_OK) {
        return CAN_STEPPERS_CALIBRATION_FAIL;
    }

    // 5.1 Wait calibration result
    std::vector<StepperMotorState*> stepper = { &m3 };
    if (getCalibrationResults(stepper, calibration_timeout) != CAN_STEPPERS_CALIBRATION_OK) {
        return CAN_STEPPERS_CALIBRATION_FAIL;
    }

    return CAN_STEPPERS_CALIBRATION_OK;
}

void CanCommunication::setGoalPosition(double axis_1_pos_goal, double axis_2_pos_goal, double axis_3_pos_goal, double axis_4_pos_goal)
{
    m1.setPositionCommand(rad_pos_to_steps(axis_1_pos_goal, m1.getGearRatio(), m1.getDirection()));
    m2.setPositionCommand(rad_pos_to_steps(axis_2_pos_goal, m2.getGearRatio(), m2.getDirection()));
    m3.setPositionCommand(rad_pos_to_steps(axis_3_pos_goal, m3.getGearRatio(), m3.getDirection()));
    m4.setPositionCommand(rad_pos_to_steps(axis_4_pos_goal, m4.getGearRatio(), m4.getDirection()));

    // if motor disabled, pos_state = pos_cmd (echo position)
    for (int i = 0 ; i < motors.size(); i++) {
        if (!motors.at(i)->isEnabled()) {
            motors.at(i)->setPositionState(motors.at(i)->getPositionCommand());
        }
    }
}

void CanCommunication::getCurrentPosition(double *axis_1_pos, double *axis_2_pos, double *axis_3_pos, double *axis_4_pos)
{
    *axis_1_pos = steps_to_rad_pos(m1.getPositionState(), m1.getGearRatio(), m1.getDirection());
    *axis_2_pos = steps_to_rad_pos(m2.getPositionState(), m2.getGearRatio(), m2.getDirection());
    *axis_3_pos = steps_to_rad_pos(m3.getPositionState(), m3.getGearRatio(), m3.getDirection());
    *axis_4_pos = steps_to_rad_pos(m4.getPositionState(), m4.getGearRatio(), m4.getDirection());
}

void CanCommunication::setMicroSteps(std::vector<uint8_t> micro_steps_list)
{
    if (micro_steps_list.size() != 4) {
        ROS_WARN("Micro steps array must have 4 values");
        return;
    }

    for (int i = 0; i < micro_steps_list.size(); i++) {
        motors.at(i)->setMicroStepsCommand(micro_steps_list.at(i));
    }

    write_micro_steps_enable = true;
}

void CanCommunication::setMaxEffort(std::vector<uint8_t> max_effort_list)
{
    if (max_effort_list.size() != 4) {
        ROS_WARN("Max effort array must have 4 values");
        return;
    }

    for (int i = 0; i < max_effort_list.size(); i++) {
        motors.at(i)->setMaxEffortCommand(max_effort_list.at(i));
    }

    write_max_effort_enable = true;
}

void CanCommunication::getHardwareStatus(bool *is_connection_ok, std::string &error_message, 
        int *calibration_needed, bool *calibration_in_progress,
        std::vector<std::string> &motor_names, std::vector<std::string> &motor_types,
        std::vector<int32_t> &temperatures, std::vector<double> &voltages,
        std::vector<int32_t> &hw_errors)
{
    *(is_connection_ok) = is_can_connection_ok;
    *(calibration_needed) = waiting_for_user_trigger_calibration; 
    *(calibration_in_progress) = this->calibration_in_progress;
    error_message = debug_error_message;

    motor_names.clear();
    motor_types.clear();
    temperatures.clear();
    voltages.clear();
    hw_errors.clear();

    for (int i = 0 ; i < motors.size(); i++) {
        motor_names.push_back(motors.at(i)->getName());
        motor_types.push_back("niryo_stepper");
        temperatures.push_back(motors.at(i)->getTemperatureState());
        voltages.push_back(0.0);
        hw_errors.push_back(motors.at(i)->getHardwareErrorState());
    }
}

void CanCommunication::getFirmwareVersions(std::vector<std::string> &motor_names,
        std::vector<std::string> &firmware_versions)
{
    motor_names.clear();
    firmware_versions.clear();

    for (int i = 0; i < motors.size(); i++) {
        motor_names.push_back(motors.at(i)->getName());
        firmware_versions.push_back(motors.at(i)->getFirmwareVersion());
    }
}

bool CanCommunication::isConnectionOk()
{
    return is_can_connection_ok;
}

/*
 * Will wait for all required motors to send one can frame
 * --> error when
 *  - a motor id is not allowed
 *  - a required motor is missing
 *  - can't get access to CAN bus
 */
int CanCommunication::scanAndCheck()
{
    int counter = 0;

    while (hw_is_busy && counter < 100) { 
        ros::Duration(TIME_TO_WAIT_IF_BUSY).sleep();    
        counter++;
    }
    
    if (counter == 100) {
        debug_error_message = "Failed to scan motors, CAN bus is too busy. Will retry...";
        ROS_WARN("Failed to scan motors, CAN bus is too busy (counter max : %d)", counter);
        return CAN_SCAN_BUSY;
    }
   
    hw_is_busy = true;
    
    // if some motors are disabled, just declare them as connected
    bool m1_ok = !m1.isEnabled(); 
    bool m2_ok = !m2.isEnabled();
    bool m3_ok = !m3.isEnabled();
    bool m4_ok = !m4.isEnabled();
   
    double time_begin_scan = ros::Time::now().toSec();
    double timeout = 0.5;

    while (!m1_ok || !m2_ok || !m3_ok || !m4_ok) {
        ros::Duration(0.001).sleep(); // check at 1000 Hz
        
        if (can->canReadData()) {
            long unsigned int rxId;
            unsigned char len;
            unsigned char rxBuf[8];
            
            can->readMsgBuf(&rxId, &len, rxBuf);
            
            // Validate id
            int motor_id = rxId & 0x00F; // 0x101 for id 1, 0x102 for id 2, ...
            if (motor_id == m1.getId()) {
                m1_ok = true;
            }
            else if (motor_id == m2.getId()) {
                m2_ok = true;
            }
            else if (motor_id == m3.getId()) {
                m3_ok = true;
            }
            else if (motor_id == m4.getId()) {
                m4_ok = true;
            }
            else { // detect unallowed motor
                ROS_ERROR("Received can frame with wrong id : %d", motor_id);
                hw_is_busy = false;
                debug_error_message = "Unknown connected motor : ";
                debug_error_message = std::to_string(motor_id);
                return CAN_SCAN_NOT_ALLOWED;
            }
        }

        if (ros::Time::now().toSec() - time_begin_scan > timeout) {
            ROS_ERROR("CAN SCAN Timeout");
            debug_error_message = "CAN bus scan failed : motors ";
            if (!m1_ok) { debug_error_message += std::to_string(m1.getId()); debug_error_message += ", "; }
            if (!m2_ok) { debug_error_message += std::to_string(m2.getId()); debug_error_message += ", "; }
            if (!m3_ok) { debug_error_message += std::to_string(m3.getId()); debug_error_message += ", "; }
            if (!m4_ok) { debug_error_message += std::to_string(m4.getId()); debug_error_message += ", "; }
            debug_error_message += "are not connected";
            is_can_connection_ok = false;
            hw_is_busy = false;
            return CAN_SCAN_TIMEOUT;
        }
    }

    //ROS_INFO("CAN Connection ok");
    hw_is_busy = false;
    is_can_connection_ok = true;
    debug_error_message = "";
    return CAN_SCAN_OK;
}


