# Test MiraAuthority
ament_add_gtest(test_mira_authority test_mira_authority.cpp)
target_link_libraries(test_mira_authority
  scitos2_mira_utils
)
