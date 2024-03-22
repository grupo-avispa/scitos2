## THIS MESSAGE IS DEPRECATED AS OF GALACTIC
## Please use sensor_msgs/BatteryState instead.

# This message holds the current battery state. 
# The battery state is described by the current voltage, the battery current, the charge state
# in percent, an estimated life time in minutes and a flag, which indicates if the battery is
# currently charged or not.

std_msgs/Header header
float32 voltage                 # The battery voltage in [V].
float32 current                 # The battery current in [A].
int8 life_percent               # The battery life percent [0..100] or 255 if unknown.
int16 life_time                 # The battery life time in minutes or -1 if unknown.
bool charging                   # True if the battery is currently being charged, false otherwise.
bool power_supply_present       # True if the robot is powered by an external power supply, false otherwise.
float32[] cell_voltage          # The voltage of the single battery cells in [V].
