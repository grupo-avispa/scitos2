# Test MiraAuthority
ament_add_gtest(test_mira_authority test_mira_authority.cpp)
target_link_libraries(test_mira_authority
  scitos2_mira_utils
)

# Test MiraModule
ament_add_gtest(test_mira_module
  test_mira_module.cpp
  ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp
)
target_link_libraries(test_mira_module
  scitos2_mira_utils
)
