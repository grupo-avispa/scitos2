# Test for charger
ament_add_gtest(test_charger
  test_charger.cpp
)
target_link_libraries(test_charger
  scitos2_charger
)

# Test for display
ament_add_gtest(test_display
  test_display.cpp
)
target_link_libraries(test_display
  scitos2_display
)

# Test for drive
ament_add_gtest(test_drive
  test_drive.cpp
)
target_link_libraries(test_drive
  nav2_ros_common::nav2_ros_common
  scitos2_drive
)

# Test for ebc
ament_add_gtest(test_ebc
  test_ebc.cpp
)
target_link_libraries(test_ebc
  scitos2_ebc
)

# Test for imu
ament_add_gtest(test_imu
  test_imu.cpp
)
target_link_libraries(test_imu
  scitos2_imu
)
