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
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(pluginlib REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(angles REQUIRED)
find_package(nav2_util REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(scitos2_msgs REQUIRED)
find_package(opennav_docking_core REQUIRED)
find_package(opennav_docking REQUIRED)

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
  rclcpp_components
  pluginlib
  geometry_msgs
  sensor_msgs
  angles
  nav2_util
  pcl_ros
  pcl_conversions
  tf2_ros
  scitos2_msgs
  opennav_docking_core
  opennav_docking
)

set(library_name ${PROJECT_NAME}_core)
set(dock_saver_executable dock_saver)

# Add library
add_library(${library_name} SHARED
  src/segmentation.cpp
  src/perception.cpp
  src/charging_dock.cpp
  src/dock_saver.cpp
)
ament_target_dependencies(${library_name} ${dependencies})

pluginlib_export_plugin_description_file(opennav_docking_core plugins.xml)

# Add dock saver executable
add_executable(${dock_saver_executable} src/main_saver.cpp)
ament_target_dependencies(${dock_saver_executable} ${dependencies})
target_link_libraries(${dock_saver_executable} ${library_name})

rclcpp_components_register_nodes(${library_name} "scitos2_charging_dock::DockSaver")

# ############
# # Install ##
# ############
install(TARGETS ${library_name}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS ${dock_saver_executable}
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
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
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME} ${library_name})
ament_export_dependencies(${dependencies})
ament_package()
