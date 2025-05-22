include_directories(.)

add_subdirectory(action)
add_subdirectory(condition)

# Test register
ament_add_gtest(test_register
  test_register.cpp
)
target_link_libraries(test_register
  behaviortree_cpp::behaviortree_cpp
  rclcpp::rclcpp
)
