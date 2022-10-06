# This message holds the current hardware state.

std_msgs/Header header
bool mode_normal            # Accumulated state that signals whether the robot is in a normal operation mode
bool mode_forced_stopped    # This bit signals that a forced motor stop is present
bool mode_freerun           # Free run reset
bool mode_safe_drive        # Safe / slow drive mode active
bool mode_bumper_escape     # True if bumper escape mode is active
                            # The bumper escape mode is used to release the mode_forced_stopped caused by a bumper

bool error_sifas_communication      # I2C_error sifas
bool error_sifas_internal           # Bit that accumulates different sifas errors
bool emergency_stop_activated       # Bit that signals whether a emergency stop is active
                                    # An emergency stop is active as soon as emergency_stop_activated was raised and persists till resetMotorStop is called.
bool emergency_stop_status          # The current state of the emergency stop
bool bumper_front_activated         # Bit that signals if the front bumper is active
                                    # The bumper is considered active for about 3s after the last observed bumper_front_activated
bool bumper_front_status            # The current status of the front bumper
bool bumper_rear_activated          # Bit that signals if the rear bumper is active
                                    # The bumper is considered active for about 3s after the last observed bumper_rear_activated
bool bumper_rear_status             # The current status of the rear bumper
bool magnetic_proximity_sensor_activated    # Bit that signals whether a magnetic sensor is active
bool magnetic_proximity_sensor_status       # The current state of the magnetic sensor
bool external_safety_ctrl_status

bool error_battery_low
bool error_temperature_high
bool error_stall_mode
bool limited_pwm_left
bool limited_pwm_right
bool error_rotation_too_high        # Rotation error: too fast rotation
bool error_motor_acceleration       # Motor moment: too high motor acceleration
bool error_motor_left               # Motor sensors left: Communication error with left motor
bool error_motor_right              # Motor sensor right: Communication error with right motor
bool safety_field_rear_laser        # Laser back: Safety field back laser active (needs firmware >= 1.0.3)
bool safety_field_front_laser       # Laser front: Safety field front laser active (needs firmware >= 1.0.3)
bool error_firmare                  # Firmware error
bool relay_active                   # Relay is active. Robot can't drive.
bool software_safe_stop
