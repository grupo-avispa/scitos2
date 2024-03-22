## THIS MESSAGE IS DEPRECATED AS OF GALACTIC
## Please use sensor_msgs/BatteryState instead.

# This message holds the state of the charger.

std_msgs/Header header
bool charging               # True if the charger is currently charging, false otherwise.
bool empty                  # True if the battery is empty, false otherwise.
bool full                   # True if the battery is full, false otherwise.
bool derating               # Bit 3: 1=derating, 0=not derating (unused)
bool charging_disabled      # (only intern)
bool const_volt_mode        # True if the charger is in constant voltage mode, false otherwise.
bool const_current_mode     # True if the charger is in constant current mode, false otherwise.
bool internal_error_flag    # True if the charger has an internal error, false otherwise.

