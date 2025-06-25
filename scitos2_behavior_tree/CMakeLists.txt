cmake_minimum_required(VERSION 3.5)
project(scitos2_behavior_tree)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to Release as none was specified.")
  set(CMAKE_BUILD_TYPE "Release" CACHE
    STRING "Choose the type of build." FORCE)

  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  if("cxx_std_17" IN_LIST CMAKE_CXX_COMPILE_FEATURES)
    set(CMAKE_CXX_STANDARD 17)
  else()
    message(FATAL_ERROR "cxx_std_17 could not be found.")
  endif()
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wdeprecated -fPIC -Wnull-dereference)
  add_compile_options("$<$<COMPILE_LANGUAGE:CXX>:-Wnon-virtual-dtor>")
endif()

option(COVERAGE_ENABLED "Enable code coverage" FALSE)

if(COVERAGE_ENABLED)
  add_compile_options(--coverage)
  set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --coverage")
  set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} --coverage")
endif()

# Defaults for Microsoft C++ compiler
if(MSVC)
  # https://blog.kitware.com/create-dlls-on-windows-without-declspec-using-new-cmake-export-all-feature/
  set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

  # Enable Math Constants
  # https://docs.microsoft.com/en-us/cpp/c-runtime-library/math-constants?view=vs-2019
  add_compile_definitions(
    _USE_MATH_DEFINES
  )
endif()

# ###############################################
# # Find  dependencies                         ##
# ###############################################
# # Find ament macros and libraries
find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(nav2_behavior_tree REQUIRED)
find_package(nav2_ros_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(scitos2_msgs REQUIRED)

# ##########
# # Build ##
# ##########
add_library(scitos2_is_bumper_activated_condition_bt_node SHARED src/condition/is_bumper_activated_condition.cpp)
target_include_directories(scitos2_is_bumper_activated_condition_bt_node PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  "$<BUILD_INTERFACE:${nav2_ros_common_INCLUDE_DIRS}>"
)
target_link_libraries(scitos2_is_bumper_activated_condition_bt_node
  PUBLIC
  behaviortree_cpp::behaviortree_cpp
  rclcpp::rclcpp
  ${scitos2_msgs_TARGETS}
)
target_compile_definitions(scitos2_is_bumper_activated_condition_bt_node PRIVATE BT_PLUGIN_EXPORT)

add_library(scitos2_emergency_stop_service_bt_node SHARED src/action/emergency_stop_service.cpp)
target_include_directories(scitos2_emergency_stop_service_bt_node PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  "$<BUILD_INTERFACE:${nav2_ros_common_INCLUDE_DIRS}>"
)
target_link_libraries(scitos2_emergency_stop_service_bt_node
  PUBLIC
  behaviortree_cpp::behaviortree_cpp
  nav2_behavior_tree::nav2_behavior_tree
  ${scitos2_msgs_TARGETS}
)
target_compile_definitions(scitos2_emergency_stop_service_bt_node PRIVATE BT_PLUGIN_EXPORT)

add_library(scitos2_reset_motor_stop_service_bt_node SHARED src/action/reset_motor_stop_service.cpp)
target_include_directories(scitos2_reset_motor_stop_service_bt_node PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
  "$<BUILD_INTERFACE:${nav2_ros_common_INCLUDE_DIRS}>"
)
target_link_libraries(scitos2_reset_motor_stop_service_bt_node
  PUBLIC
  nav2_behavior_tree::nav2_behavior_tree
  ${scitos2_msgs_TARGETS}
)
target_compile_definitions(scitos2_reset_motor_stop_service_bt_node PRIVATE BT_PLUGIN_EXPORT)

# ############
# # Install ##
# ############
install(TARGETS
  scitos2_is_bumper_activated_condition_bt_node
  scitos2_emergency_stop_service_bt_node
  scitos2_reset_motor_stop_service_bt_node
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# we will embed the list of plugin names inside a header file
set(GENERATED_DIR ${CMAKE_CURRENT_BINARY_DIR}/gen)
configure_file(plugins_list.hpp.in ${GENERATED_DIR}/plugins_list.hpp)

add_executable(generate_scitos2_tree_nodes_xml src/generate_scitos2_tree_nodes_xml.cpp)
target_link_libraries(generate_scitos2_tree_nodes_xml PRIVATE
  behaviortree_cpp::behaviortree_cpp
)

# allow generate_scitos2_tree_nodes_xml to find plugins_list.hpp
target_include_directories(generate_scitos2_tree_nodes_xml PRIVATE ${GENERATED_DIR})
install(TARGETS generate_scitos2_tree_nodes_xml DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
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
  find_package(ament_cmake_gtest REQUIRED)
  add_subdirectory(test)
endif()

# ##################################
# # ament specific configuration  ##
# ##################################
ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(
  scitos2_is_bumper_activated_condition_bt_node
  scitos2_emergency_stop_service_bt_node
  scitos2_reset_motor_stop_service_bt_node
)
ament_export_dependencies(
  behaviortree_cpp
  nav2_behavior_tree
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  scitos2_msgs
)
ament_export_targets(${PROJECT_NAME})
ament_package()
