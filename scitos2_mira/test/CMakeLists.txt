# Test mira framework
ament_add_gtest(test_mira_framework
  test_mira_framework.cpp
)
ament_target_dependencies(test_mira_framework
  ${dependencies}
)
target_link_libraries(test_mira_framework
  ${library_name}
)