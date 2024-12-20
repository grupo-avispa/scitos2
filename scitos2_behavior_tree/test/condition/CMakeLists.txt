find_package(geometry_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# Test for is bumper activated condition
ament_add_gtest(test_scitos2_condition_is_bumper_activated
  test_is_bumper_activated_condition.cpp
)
target_link_libraries(test_scitos2_condition_is_bumper_activated
  ${geometry_msgs_TARGETS}
  scitos2_is_bumper_activated_condition_bt_node
  ${scitos2_msgs_TARGETS}
  rclcpp_action::rclcpp_action
  ${tf2_msgs_TARGETS}
  tf2_ros::tf2_ros
)