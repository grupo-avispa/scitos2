cmake_minimum_required(VERSION 3.8)
project(scitos_mira)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################
## Find  dependencies                         ##
################################################
## Find ament macros and libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(scitos_msgs REQUIRED)

## Find system libraries
find_package(Boost REQUIRED COMPONENTS thread)

###########
## Build ##
###########
## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${Boost_INCLUDE_DIRS}
)

# Include cmake file for MIRA
include(CMakeLists.mira)
MIRA_REQUIRE_PACKAGE(SCITOS)

add_executable(${PROJECT_NAME}
  src/ScitosCharger.cpp
  src/ScitosDrive.cpp
  src/ScitosModule.cpp
  src/scitos_node.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  scitos_msgs
)
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

##########
## MIRA ##
##########

# For mira support (auto linking, enabling C++14 support,...)
cmake_add_mira(${PROJECT_NAME})

#############
## Install ##
#############
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.hpp"
#   PATTERN ".svn" EXCLUDE
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

#############
## Testing ##
#############
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()