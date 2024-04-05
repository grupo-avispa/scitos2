# Test for charger
ament_add_gtest(test_charger
  test_charger.cpp
)
ament_target_dependencies(test_charger
  ${dependencies}
)
target_link_libraries(test_charger
  scitos2_charger
)

# Test for display
ament_add_gtest(test_display
  test_display.cpp
)
ament_target_dependencies(test_display
  ${dependencies}
)
target_link_libraries(test_display
  scitos2_display
)

# Test for drive
ament_add_gtest(test_drive
  test_drive.cpp
)
ament_target_dependencies(test_drive
  ${dependencies}
)
target_link_libraries(test_drive
  scitos2_drive
)

# Test for ebc
ament_add_gtest(test_ebc
  test_ebc.cpp
)
ament_target_dependencies(test_ebc
  ${dependencies}
)
target_link_libraries(test_ebc
  scitos2_ebc
)
