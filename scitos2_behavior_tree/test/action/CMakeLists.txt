find_package(nav2_behavior_tree REQUIRED)

# Test for the emergency stop service
ament_add_gtest(test_scitos2_action_emergency_stop_service
  test_emergency_stop_service.cpp
)
ament_target_dependencies(test_scitos2_action_emergency_stop_service
  ${dependencies}
)
target_link_libraries(test_scitos2_action_emergency_stop_service
  scitos2_emergency_stop_service_bt_node
)

# Test for the reset motor stop service
ament_add_gtest(test_scitos2_action_reset_motor_stop_service
  test_reset_motor_stop_service.cpp
)
ament_target_dependencies(test_scitos2_action_reset_motor_stop_service
  ${dependencies}
)
target_link_libraries(test_scitos2_action_reset_motor_stop_service
  scitos2_reset_motor_stop_service_bt_node
)