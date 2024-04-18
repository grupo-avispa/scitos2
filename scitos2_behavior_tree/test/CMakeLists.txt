include_directories(.)

add_subdirectory(action)
add_subdirectory(condition)

# Test register
ament_add_gtest(test_register
test_register.cpp
)
ament_target_dependencies(test_register
  ${dependencies}
)
