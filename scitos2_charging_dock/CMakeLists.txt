cmake_minimum_required(VERSION 3.5)
project(scitos2_charging_dock)

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
  add_compile_options(-Wall -Wextra -Wpedantic -Werror -Wdeprecated -fPIC -Wshadow -Wnull-dereference)
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
find_package(angles REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_ros_common REQUIRED)
find_package(nav2_util REQUIRED)
find_package(opennav_docking_core REQUIRED)
find_package(opennav_docking REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(scitos2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

# ##########
# # Build ##
# ##########
# # Specify additional locations of header files
# # Your package locations should be listed before other locations
set(library_name ${PROJECT_NAME}_core)
set(dock_saver_executable dock_saver)

# Add segmentation library
add_library(segmentation SHARED src/segmentation.cpp)
target_include_directories(segmentation PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(segmentation
  PUBLIC
  ${geometry_msgs_TARGETS}
  rclcpp::rclcpp
  rclcpp_lifecycle::rclcpp_lifecycle
  ${sensor_msgs_TARGETS}
  PRIVATE
  nav2_ros_common::nav2_ros_common
  pcl_ros::pcl_ros_tf
)

# Add perception library
add_library(perception SHARED src/perception.cpp)
target_include_directories(perception PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(perception
  PUBLIC
  ${geometry_msgs_TARGETS}
  ${PCL_LIBRARIES}
  rclcpp::rclcpp
  segmentation
  ${sensor_msgs_TARGETS}
  tf2_ros::tf2_ros
  PRIVATE
  nav2_ros_common::nav2_ros_common
  pcl_ros::pcl_ros_tf
  pcl_conversions::pcl_conversions
  ${tf2_geometry_msgs_TARGETS}
)

# Add charging dock library
add_library(${library_name} SHARED src/charging_dock.cpp)
target_include_directories(${library_name} PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(${library_name}
  PUBLIC
  ${geometry_msgs_TARGETS}
  perception
  opennav_docking_core::opennav_docking_core
  opennav_docking::pose_filter
  rclcpp_lifecycle::rclcpp_lifecycle
  ${scitos2_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  tf2_ros::tf2_ros
  PRIVATE
  angles::angles
  nav2_ros_common::nav2_ros_common
  nav2_util::nav2_util_core
  pluginlib::pluginlib
  ${tf2_geometry_msgs_TARGETS}
)

pluginlib_export_plugin_description_file(opennav_docking_core plugins.xml)

# Add dock saver library
add_library(dock_saver_core SHARED src/dock_saver.cpp)
target_include_directories(dock_saver_core PUBLIC
  "$<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)
target_link_libraries(dock_saver_core
  PUBLIC
  nav2_ros_common::nav2_ros_common
  perception
  rclcpp::rclcpp
  ${scitos2_msgs_TARGETS}
  tf2_ros::tf2_ros
  PRIVATE
  angles::angles
  ${sensor_msgs_TARGETS}
  rclcpp_components::component
)

# Add dock saver executable
add_executable(${dock_saver_executable} src/main_saver.cpp)
target_link_libraries(${dock_saver_executable}
  PRIVATE
  dock_saver_core
)

rclcpp_components_register_nodes(${library_name} "scitos2_charging_dock::DockSaver")

# ############
# # Install ##
# ############
install(TARGETS ${library_name}
  segmentation
  perception
  dock_saver_core
  EXPORT ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${dock_saver_executable}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(DIRECTORY launch params
  DESTINATION share/${PROJECT_NAME}
)

install(FILES test/dock_test.pcd test/empty_dock_test.pcd
  DESTINATION share/${PROJECT_NAME}/test
)

# ############
# # Testing ##
# ############
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
  ${library_name}
  segmentation
  perception
  dock_saver_core
)
ament_export_dependencies(
  angles
  geometry_msgs
  nav2_ros_common
  opennav_docking_core
  opennav_docking
  pcl_ros
  pcl_conversions
  pluginlib
  rclcpp
  rclcpp_components
  rclcpp_lifecycle
  scitos2_msgs
  sensor_msgs
  tf2_ros
)
ament_export_targets(${PROJECT_NAME})
ament_package()
