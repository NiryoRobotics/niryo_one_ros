#!/usr/bin/env python

# ! You need to launch the server first !

from pymodbus.client.sync import ModbusTcpClient
import time

print "--- START"
client = ModbusTcpClient('localhost', port=5020)

client.connect()
print "Connected to modbus server"

rr = client.read_input_registers(400, 9)
print rr.registers

rr = client.read_discrete_inputs(100, 6)
print rr.bits

# Set digital IO mode - output
client.write_coil(0, False)
client.write_coil(3, False)

# Set digital IO state
client.write_coil(100, True)
client.write_coil(103, False)

time.sleep(0.1)

rr = client.read_discrete_inputs(100, 6)
print rr.bits

client.close()
print "Close connection to modbus server"
print "--- END"
