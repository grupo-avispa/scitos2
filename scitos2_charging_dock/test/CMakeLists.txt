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