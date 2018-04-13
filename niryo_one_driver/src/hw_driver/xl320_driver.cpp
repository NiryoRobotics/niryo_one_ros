/*
    xl320_driver.cpp
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

#include "niryo_one_driver/xl320_driver.h"

XL320Driver::XL320Driver(std::string deviceName, int baudRate)
{
    this->deviceName = deviceName;
    this->baudRate = baudRate;

    portHandler = dynamixel::PortHandler::getPortHandler(deviceName.c_str());
    packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
}

int XL320Driver::init() 
{
    // setup half-duplex direction GPIO
    // see schema http://support.robotis.com/en/product/actuator/dynamixel_x/xl-series_main.htm
    if (!portHandler->setupGpio()) {
        return DXL_FAIL_SETUP_GPIO;
    }

    // Open port
    if (!portHandler->openPort()) {
        return DXL_FAIL_OPEN_PORT;
    }

    // Set baudrate
    if (!portHandler->setBaudRate(baudRate)) {
        return DXL_FAIL_PORT_SET_BAUDRATE;
    }
    
    return 0;
}

int XL320Driver::ping(uint8_t id) 
{
    uint8_t dxl_error = 0;
    uint16_t dxl_model_number = 0;

    int result = packetHandler->ping(portHandler, id, &dxl_model_number, &dxl_error);

    if (dxl_error != 0) {
        return dxl_error; 
    }
    
    if (result == COMM_SUCCESS) {
        if (dxl_model_number && dxl_model_number != DXL_MODEL_NUMBER) {
            return PING_WRONG_MODEL_NUMBER;
        }
    }

    return result;
}

int XL320Driver::scan(std::vector<uint8_t> &id_list) 
{
    return packetHandler->broadcastPing(portHandler, id_list);
}

/*
 *  -----------------   WRITE   --------------------
 */

int XL320Driver::changeId(uint8_t id, uint8_t new_id)
{
    return packetHandler->write1ByteTxOnly(portHandler, id, DXL_ADDR_ID, new_id);
}

int XL320Driver::changeBaudRate(uint8_t id, uint32_t new_baudrate)
{
    return packetHandler->write1ByteTxOnly(portHandler, id, DXL_ADDR_BAUDRATE, (uint8_t)new_baudrate);    
}

int XL320Driver::setLed(uint8_t id, uint32_t led_value)
{
    return packetHandler->write1ByteTxOnly(portHandler, id, DXL_ADDR_LED, (uint8_t)led_value);
}

int XL320Driver::setTorqueEnable(uint8_t id, uint32_t torque_enable)
{
    return packetHandler->write1ByteTxOnly(portHandler, id, DXL_ADDR_TORQUE_ENABLE, (uint8_t)torque_enable);
}

int XL320Driver::setGoalPosition(uint8_t id, uint32_t position)
{
    return packetHandler->write2ByteTxOnly(portHandler, id, DXL_ADDR_GOAL_POSITION, (uint16_t)position);
}
        
int XL320Driver::setGoalVelocity(uint8_t id, uint32_t velocity)
{
    return packetHandler->write2ByteTxOnly(portHandler, id, DXL_ADDR_GOAL_SPEED, (uint16_t)velocity);
}

int XL320Driver::setGoalTorque(uint8_t id, uint32_t torque)
{
    return packetHandler->write2ByteTxOnly(portHandler, id, DXL_ADDR_GOAL_TORQUE, (uint16_t)torque);
}

int XL320Driver::setReturnDelayTime(uint8_t id, uint32_t return_delay_time)
{
    return packetHandler->write1ByteTxOnly(portHandler, id, DXL_ADDR_RETURN_DELAY_TIME, (uint8_t)return_delay_time);
}

int XL320Driver::setLimitTemperature(uint8_t id, uint32_t temperature)
{
    return packetHandler->write1ByteTxOnly(portHandler, id, DXL_ADDR_LIMIT_TEMPERATURE, (uint8_t)temperature);
}

int XL320Driver::setMaxTorque(uint8_t id, uint32_t torque)
{
    return packetHandler->write1ByteTxOnly(portHandler, id, DXL_ADDR_MAX_TORQUE, (uint8_t)torque);
}

int XL320Driver::setReturnLevel(uint8_t id, uint32_t return_level)
{
    return packetHandler->write1ByteTxOnly(portHandler, id, DXL_ADDR_RETURN_LEVEL, (uint8_t)return_level);
}

int XL320Driver::setAlarmShutdown(uint8_t id, uint32_t alarm_shutdown)
{
    return packetHandler->write1ByteTxOnly(portHandler, id, DXL_ADDR_ALARM_SHUTDOWN, (uint8_t)alarm_shutdown);
}

/*
 *  -----------------   SYNC WRITE   --------------------
 */

int XL320Driver::syncWrite1Byte(uint8_t address, std::vector<uint8_t> &id_list, std::vector<uint32_t> &data_list)
{
    dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, address, DXL_LEN_ONE_BYTE);

    if (id_list.size() != data_list.size()) {
        return LEN_ID_DATA_NOT_SAME; 
    }

    if (id_list.size() == 0) {
        return COMM_SUCCESS;
    }
    
    std::vector<uint8_t>::iterator it_id;
    std::vector<uint32_t>::iterator it_data;

    for (it_id=id_list.begin(), it_data=data_list.begin() ; 
            it_id < id_list.end() && it_data < data_list.end() ; 
            it_id++, it_data++) {
        uint8_t params[1] = { (uint8_t)(*it_data) };
        if (!groupSyncWrite.addParam(*it_id, params)) {
            groupSyncWrite.clearParam();
            return GROUP_SYNC_REDONDANT_ID; 
        }
    }
    
    int dxl_comm_result = groupSyncWrite.txPacket();
    groupSyncWrite.clearParam();
    return dxl_comm_result;
}

int XL320Driver::syncWrite2Bytes(uint8_t address, std::vector<uint8_t> &id_list, std::vector<uint32_t> &data_list)
{
    dynamixel::GroupSyncWrite groupSyncWrite(portHandler, packetHandler, address, DXL_LEN_TWO_BYTES);

    if (id_list.size() != data_list.size()) {
        return LEN_ID_DATA_NOT_SAME; 
    }
    
    if (id_list.size() == 0) {
        return COMM_SUCCESS;
    }
    
    std::vector<uint8_t>::iterator it_id;
    std::vector<uint32_t>::iterator it_data;

    for (it_id=id_list.begin(), it_data=data_list.begin() ; 
            it_id < id_list.end() && it_data < data_list.end() ; 
            it_id++, it_data++) {
        uint8_t params[2] = { DXL_LOBYTE((uint16_t)(*it_data)), DXL_HIBYTE((uint16_t)(*it_data)) };
        if (!groupSyncWrite.addParam(*it_id, params)) {
            groupSyncWrite.clearParam();
            return GROUP_SYNC_REDONDANT_ID;
        }
    }
    
    int dxl_comm_result = groupSyncWrite.txPacket();
    groupSyncWrite.clearParam();
    return dxl_comm_result;
}

int XL320Driver::syncWritePositionGoal(std::vector<uint8_t> &id_list, std::vector<uint32_t> &position_list)
{
    return syncWrite2Bytes(DXL_ADDR_GOAL_POSITION, id_list, position_list);
}
        
int XL320Driver::syncWriteVelocityGoal(std::vector<uint8_t> &id_list, std::vector<uint32_t> &velocity_list)
{
    return syncWrite2Bytes(DXL_ADDR_GOAL_SPEED, id_list, velocity_list);
}
        
int XL320Driver::syncWriteTorqueGoal(std::vector<uint8_t> &id_list, std::vector<uint32_t> &torque_list)
{
    return syncWrite2Bytes(DXL_ADDR_GOAL_TORQUE, id_list, torque_list);
}

int XL320Driver::syncWriteTorqueEnable(std::vector<uint8_t> &id_list, std::vector<uint32_t> &torque_enable_list) 
{
    return syncWrite1Byte(DXL_ADDR_TORQUE_ENABLE, id_list, torque_enable_list);
}

int XL320Driver::syncWriteLed(std::vector<uint8_t> &id_list, std::vector<uint32_t> &led_list)
{
    return syncWrite1Byte(DXL_ADDR_LED, id_list, led_list);
}

/*
 *  -----------------   READ   --------------------
 */

int XL320Driver::read1Byte(uint8_t address, uint8_t id, uint32_t *data)
{
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;

    uint8_t read_data;
    dxl_comm_result = packetHandler->read1ByteTxRx(portHandler, id, address, &read_data, &dxl_error);
    (*data) = read_data;

    if (dxl_error != 0) {
        return dxl_error;
    }

    return dxl_comm_result;
}

int XL320Driver::read2Bytes(uint8_t address, uint8_t id, uint32_t *data)
{
    uint8_t dxl_error = 0;
    int dxl_comm_result = COMM_TX_FAIL;

    uint16_t read_data;
    dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, id, address, &read_data, &dxl_error);
    (*data) = read_data;

    if (dxl_error != 0) {
        return dxl_error;
    }

    return dxl_comm_result;
}

int XL320Driver::readPosition(uint8_t id, uint32_t *present_position)
{
    return read2Bytes(DXL_ADDR_PRESENT_POSITION, id, present_position);
}

int XL320Driver::readVelocity(uint8_t id, uint32_t *present_velocity)
{
    return read2Bytes(DXL_ADDR_PRESENT_SPEED, id, present_velocity);
}

int XL320Driver::readLoad(uint8_t id, uint32_t *present_load)
{
    return read2Bytes(DXL_ADDR_PRESENT_LOAD, id, present_load);
}

int XL320Driver::readTemperature(uint8_t id, uint32_t *temperature)
{
    return read1Byte(DXL_ADDR_PRESENT_TEMPERATURE, id, temperature); 
}

int XL320Driver::readVoltage(uint8_t id, uint32_t *voltage)
{
    return read1Byte(DXL_ADDR_PRESENT_VOLTAGE, id, voltage);
}

int XL320Driver::readHardwareStatus(uint8_t id, uint32_t *hardware_status)
{
    return read1Byte(DXL_ADDR_HW_ERROR_STATUS, id, hardware_status);
}
        
int XL320Driver::readReturnDelayTime(uint8_t id, uint32_t *return_delay_time)
{
    return read1Byte(DXL_ADDR_RETURN_DELAY_TIME, id, return_delay_time);
}

int XL320Driver::readLimitTemperature(uint8_t id, uint32_t *limit_temperature)
{
    return read1Byte(DXL_ADDR_LIMIT_TEMPERATURE, id, limit_temperature);
}

int XL320Driver::readMaxTorque(uint8_t id, uint32_t *max_torque)
{
    return read2Bytes(DXL_ADDR_MAX_TORQUE, id, max_torque);
}

int XL320Driver::readReturnLevel(uint8_t id, uint32_t *return_level)
{
    return read1Byte(DXL_ADDR_RETURN_LEVEL, id, return_level);
}

int XL320Driver::readAlarmShutdown(uint8_t id, uint32_t *alarm_shutdown)
{
    return read1Byte(DXL_ADDR_ALARM_SHUTDOWN, id, alarm_shutdown);
}

/*
 *  -----------------   SYNC READ   --------------------
 */

int XL320Driver::syncRead(uint8_t address, uint8_t data_len, std::vector<uint8_t> &id_list, std::vector<uint32_t> &data_list)
{
    data_list.clear();

    dynamixel::GroupSyncRead groupSyncRead(portHandler, packetHandler, address, data_len);
    bool dxl_getdata_result = false;
    int dxl_comm_result = COMM_TX_FAIL;

    std::vector<uint8_t>::iterator it_id;

    for (it_id=id_list.begin() ; it_id < id_list.end() ; it_id++) {
        if (!groupSyncRead.addParam(*it_id)) {
            groupSyncRead.clearParam();
            return GROUP_SYNC_REDONDANT_ID;
        }
    }
    
    dxl_comm_result = groupSyncRead.txRxPacket();

    if (dxl_comm_result != COMM_SUCCESS) {
        groupSyncRead.clearParam();
        return dxl_comm_result;
    }

    for (it_id=id_list.begin() ; it_id < id_list.end() ; it_id++) {
        dxl_getdata_result = groupSyncRead.isAvailable(*it_id, address, data_len);
        if (!dxl_getdata_result) {
            groupSyncRead.clearParam();
            return GROUP_SYNC_READ_RX_FAIL;
        }
        if (data_len == DXL_LEN_ONE_BYTE) {
            data_list.push_back((uint8_t)groupSyncRead.getData(*it_id, address, data_len));
        }
        else if (data_len == DXL_LEN_TWO_BYTES) {
            data_list.push_back((uint16_t)groupSyncRead.getData(*it_id, address, data_len));
        }
    }

    groupSyncRead.clearParam();
    return dxl_comm_result;

}

int XL320Driver::syncReadPosition(std::vector<uint8_t> &id_list, std::vector<uint32_t> &position_list)
{
    return syncRead(DXL_ADDR_PRESENT_POSITION, DXL_LEN_TWO_BYTES, id_list, position_list);
}

int XL320Driver::syncReadVelocity(std::vector<uint8_t> &id_list, std::vector<uint32_t> &velocity_list)
{
    return syncRead(DXL_ADDR_PRESENT_SPEED, DXL_LEN_TWO_BYTES, id_list, velocity_list);
}
        
int XL320Driver::syncReadLoad(std::vector<uint8_t> &id_list, std::vector<uint32_t> &load_list)
{
    return syncRead(DXL_ADDR_PRESENT_LOAD, DXL_LEN_TWO_BYTES, id_list, load_list);
}

int XL320Driver::syncReadTemperature(std::vector<uint8_t> &id_list, std::vector<uint32_t> &temperature_list)
{
    return syncRead(DXL_ADDR_PRESENT_TEMPERATURE, DXL_LEN_ONE_BYTE, id_list, temperature_list);
}

int XL320Driver::syncReadVoltage(std::vector<uint8_t> &id_list, std::vector<uint32_t> &voltage_list)
{
    return syncRead(DXL_ADDR_PRESENT_VOLTAGE, DXL_LEN_ONE_BYTE, id_list, voltage_list);
}

int XL320Driver::syncReadHwErrorStatus(std::vector<uint8_t> &id_list, std::vector<uint32_t> &hw_error_list)
{
    return syncRead(DXL_ADDR_HW_ERROR_STATUS, DXL_LEN_ONE_BYTE, id_list, hw_error_list);
}








