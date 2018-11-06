#ifndef DXL_TOOLS_H
#define DXL_TOOLS_H

#include "dynamixel_sdk/dynamixel_sdk.h"
#include <string>
#include <vector>

class DxlTools {

    protected:
        dynamixel::PortHandler* portHandler;
        dynamixel::PacketHandler* packetHandler;

    public:
        DxlTools() {}
        DxlTools(dynamixel::PortHandler* portHandler, dynamixel::PacketHandler* packetHandler);

        int setupDxlBus(int baudrate);
        void broadcastPing();
        void ping(int id);
        void setRegister(int id, int reg_address, int value, int size);

        void closePort();

};

#endif
