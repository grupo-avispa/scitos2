# Test cluster
ament_add_gtest(test_scitos2_cluster
  test_cluster.cpp
)
ament_target_dependencies(test_scitos2_cluster
  ${dependencies}
)
target_link_libraries(test_scitos2_cluster
  ${library_name}
)

# Test segmentation
ament_add_gtest(test_scitos2_segmentation
  test_segmentation.cpp
)
ament_target_dependencies(test_scitos2_segmentation
  ${dependencies}
)
target_link_libraries(test_scitos2_segmentation
  ${library_name}
)

# Test perception
ament_add_gtest(test_scitos2_perception
  test_perception.cpp
)
ament_target_dependencies(test_scitos2_perception
  ${dependencies}
)
target_link_libraries(test_scitos2_perception
  ${library_name}
)

# Test charging dock
ament_add_gtest(test_scitos2_charging_dock
  test_charging_dock.cpp
)
ament_target_dependencies(test_scitos2_charging_dock
  ${dependencies}
)
target_link_libraries(test_scitos2_charging_dock
  ${library_name}
)

# Test dock saver
ament_add_gtest(test_dock_saver
  test_dock_saver.cpp
)
ament_target_dependencies(test_dock_saver
  ${dependencies}
)
target_link_libraries(test_dock_saver
  ${library_name}
)