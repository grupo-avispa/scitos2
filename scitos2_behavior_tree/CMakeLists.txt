cmake_minimum_required(VERSION 3.8)
project(scitos2_behavior_tree)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  if("cxx_std_17" IN_LIST CMAKE_CXX_COMPILE_FEATURES)
    set(CMAKE_CXX_STANDARD 17)
  else()
    message(FATAL_ERROR "cxx_std_17 could not be found.")
  endif()
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wdeprecated -fPIC -Wshadow -Wnull-dereference)
  add_compile_options("$<$<COMPILE_LANGUAGE:CXX>:-Wnon-virtual-dtor>")
endif()

# ###############################################
# # Find  dependencies                         ##
# ###############################################
# # Find ament macros and libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(scitos2_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(nav2_behavior_tree REQUIRED)

# ##########
# # Build ##
# ##########
# # Specify additional locations of header files
# # Your package locations should be listed before other locations
include_directories(
  include
)

set(dependencies
  rclcpp
  sensor_msgs
  scitos2_msgs
  behaviortree_cpp_v3
  nav2_behavior_tree
)

add_library(scitos2_is_bumper_activated_condition_bt_node SHARED src/condition/is_bumper_activated_condition.cpp)
list(APPEND plugin_libs scitos2_is_bumper_activated_condition_bt_node)

add_library(scitos2_emergency_stop_service_bt_node SHARED src/action/emergency_stop_service.cpp)
list(APPEND plugin_libs scitos2_emergency_stop_service_bt_node)

add_library(scitos2_reset_motor_stop_service_bt_node SHARED src/action/reset_motor_stop_service.cpp)
list(APPEND plugin_libs scitos2_reset_motor_stop_service_bt_node)

foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

# ############
# # Install ##
# ############
install(TARGETS ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# we will embed the list of plugin names inside a header file
set(GENERATED_DIR ${CMAKE_CURRENT_BINARY_DIR}/gen)
configure_file(plugins_list.hpp.in ${GENERATED_DIR}/plugins_list.hpp)

add_executable(generate_scitos2_tree_nodes_xml src/generate_scitos2_tree_nodes_xml.cpp)
ament_target_dependencies(generate_scitos2_tree_nodes_xml ${dependencies})

# allow generate_scitos2_tree_nodes_xml to find plugins_list.hpp
target_include_directories(generate_scitos2_tree_nodes_xml PRIVATE ${GENERATED_DIR})
install(TARGETS generate_scitos2_tree_nodes_xml DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/
)

install(FILES scitos2_tree_nodes.xml DESTINATION share/${PROJECT_NAME})
install(FILES ${GENERATED_DIR}/plugins_list.hpp DESTINATION include/${PROJECT_NAME})

# ###########
# Testing  ##
# ###########
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  add_subdirectory(test)
endif()

# ##################################
# # ament specific configuration  ##
# ##################################
ament_export_include_directories(include)
ament_export_libraries(${plugin_libs})
ament_export_dependencies(${dependencies})
ament_package()
