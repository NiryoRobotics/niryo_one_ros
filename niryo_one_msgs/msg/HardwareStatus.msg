
std_msgs/Header header

# Raspberry Pi board
int32 rpi_temperature 

# Motors
bool connection_up
string error_message
int32 calibration_needed
bool calibration_in_progress

string[] motor_names

int32[] temperatures
float64[] voltages
int32[] hardware_errors
