find_package(ament_index_cpp REQUIRED)

# Test cluster
ament_add_gtest(test_scitos2_cluster test_cluster.cpp)
target_include_directories(test_scitos2_cluster PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(test_scitos2_cluster
  pcl_ros::pcl_ros_tf
)

# Test segmentation
ament_add_gtest(test_scitos2_segmentation test_segmentation.cpp)
target_link_libraries(test_scitos2_segmentation
  pcl_ros::pcl_ros_tf
  rclcpp_lifecycle::rclcpp_lifecycle
  segmentation
)

# Test perception
ament_add_gtest(test_scitos2_perception test_perception.cpp)
target_link_libraries(test_scitos2_perception
  ament_index_cpp::ament_index_cpp
  nav2_ros_common::nav2_ros_common
  perception
  rclcpp::rclcpp
  tf2_ros::tf2_ros
)

# Test charging dock
ament_add_gtest(test_scitos2_charging_dock test_charging_dock.cpp)
target_link_libraries(test_scitos2_charging_dock
  ${library_name}
  nav2_ros_common::nav2_ros_common
)

# Test dock saver
ament_add_gtest(test_dock_saver test_dock_saver.cpp)
target_link_libraries(test_dock_saver
  dock_saver_core
  nav2_ros_common::nav2_ros_common
)