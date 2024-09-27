cmake_minimum_required(VERSION 3.5)
project(scitos2_modules)

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
  # add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wdeprecated -fPIC -Wshadow -Wnull-dereference)
  add_compile_options(-Wall -Wextra -Wpedantic -Wdeprecated -fPIC -Wshadow -Wnull-dereference)
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
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(scitos2_msgs REQUIRED)
find_package(scitos2_core REQUIRED)
find_package(scitos2_common REQUIRED)
find_package(pluginlib REQUIRED)

find_mira_path()

# ##########
# # Build ##
# ##########
include_mira_packages()

add_library(scitos2_charger SHARED src/charger.cpp)
target_include_directories(scitos2_charger PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(scitos2_charger
  PUBLIC
  rclcpp::rclcpp
  scitos2_core::scitos2_core
  ${scitos2_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  PRIVATE
  pluginlib::pluginlib
)
target_link_mira_libraries(scitos2_charger)

add_library(scitos2_display SHARED src/display.cpp)
target_include_directories(scitos2_display PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(scitos2_display
  PUBLIC
  rclcpp::rclcpp
  scitos2_core::scitos2_core
  ${scitos2_msgs_TARGETS}
  PRIVATE
  pluginlib::pluginlib
)
target_link_mira_libraries(scitos2_display)

add_library(scitos2_drive SHARED src/drive.cpp)
target_include_directories(scitos2_drive PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(scitos2_drive
  PUBLIC
  ${geometry_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  rclcpp::rclcpp
  scitos2_core::scitos2_core
  ${scitos2_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  tf2_ros::tf2_ros
  ${visualization_msgs_TARGETS}
  PRIVATE
  pluginlib::pluginlib
  tf2_geometry_msgs::tf2_geometry_msgs
)
ament_target_dependencies(scitos2_drive PUBLIC nav2_costmap_2d) # TODO(ajtudela): Fix this in jazzy
target_link_mira_libraries(scitos2_drive)

add_library(scitos2_ebc SHARED src/ebc.cpp)
target_include_directories(scitos2_ebc PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(scitos2_ebc
  PUBLIC
  rclcpp::rclcpp
  scitos2_core::scitos2_core
  PRIVATE
  pluginlib::pluginlib
)
target_link_mira_libraries(scitos2_ebc)

add_library(scitos2_imu SHARED src/imu.cpp)
target_include_directories(scitos2_imu PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(scitos2_imu
  PUBLIC
  ${geometry_msgs_TARGETS}
  rclcpp::rclcpp
  scitos2_core::scitos2_core
  ${sensor_msgs_TARGETS}
  PRIVATE
  pluginlib::pluginlib
)
target_link_mira_libraries(scitos2_imu)

# ############
# # Install ##
# ############
install(TARGETS
  scitos2_charger
  scitos2_display
  scitos2_drive
  scitos2_ebc
  scitos2_imu
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

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
ament_export_include_directories(include/${PROJECT_NAME})
ament_export_libraries(
  scitos2_charger
  scitos2_display
  scitos2_drive
  scitos2_ebc
  scitos2_imu
)
ament_export_dependencies(
  geometry_msgs
  nav2_costmap_2d
  nav_msgs
  pluginlib
  rclcpp
  scitos2_core
  scitos2_msgs
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  visualization_msgs
)

pluginlib_export_plugin_description_file(scitos2_core modules_plugin.xml)

ament_export_targets(${PROJECT_NAME})
ament_package()