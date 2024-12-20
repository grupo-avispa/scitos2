# This message hold the state of the magnetic barrier detection.

std_msgs/Header header
bool barrier_stopped                              # True if the barrier has been detected, false otherwise
                                                  # Reset using /reset_barrier_stop.
builtin_interfaces/Time last_detection_stamp      # Timestamp of the last detection

