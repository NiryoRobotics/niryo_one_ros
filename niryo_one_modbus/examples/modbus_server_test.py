#!/usr/bin/env python

import rospy
from niryo_one_modbus.modbus_server import ModbusServer

if __name__ == '__main__':
    rospy.init_node('modbus_server_test')

    server = ModbusServer("0.0.0.0", 5020)
    rospy.on_shutdown(server.stop)

    server.start()
    rospy.spin()
