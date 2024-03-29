cmake_minimum_required(VERSION 3.5)
project(scitos2_modules)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  if("cxx_std_17" IN_LIST CMAKE_CXX_COMPILE_FEATURES)
    set(CMAKE_CXX_STANDARD 17)
  else()
    message(FATAL_ERROR "cxx_std_17 could not be found.")
  endif()
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wdeprecated -fPIC -Wshadow -Wnull-dereference)
  # add_compile_options("$<$<COMPILE_LANGUAGE:CXX>:-Wnon-virtual-dtor>")
endif()

# ###############################################
# # Find  dependencies                         ##
# ###############################################
# # Find ament macros and libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_util REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(scitos2_msgs REQUIRED)
find_package(scitos2_core REQUIRED)
find_package(pluginlib REQUIRED)

# ##########
# # Build ##
# ##########
# # Specify additional locations of header files
# # Your package locations should be listed before other locations
include_directories(
  include
)

# Include cmake file for MIRA
include(CMakeLists.mira)
mira_require_package(SCITOS)

set(dependencies
  rclcpp
  nav2_util
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
  scitos2_msgs
  scitos2_core
  pluginlib
)

add_library(scitos2_charger SHARED src/charger.cpp)
list(APPEND plugin_libs scitos2_charger)

add_library(scitos2_display SHARED src/display.cpp)
list(APPEND plugin_libs scitos2_display)

add_library(scitos2_drive SHARED src/drive.cpp)
list(APPEND plugin_libs scitos2_drive)

add_library(scitos2_ebc SHARED src/ebc.cpp)
list(APPEND plugin_libs scitos2_ebc)

foreach(module_plugin ${plugin_libs})
  ament_target_dependencies(${module_plugin} ${dependencies})
endforeach()

# #########
# # MIRA ##
# #########

# For MIRA support (auto linking, enabling C++17 support,...)
cmake_add_mira(${plugin_libs})

# ############
# # Install ##
# ############
install(TARGETS ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

# ###########
# Testing  ##
# ###########
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  # add_subdirectory(test)
endif()

# ##################################
# # ament specific configuration  ##
# ##################################
ament_export_include_directories(include)
ament_export_libraries(${plugin_libs})
ament_export_dependencies(${dependencies})

pluginlib_export_plugin_description_file(scitos2_core modules_plugin.xml)

ament_package()