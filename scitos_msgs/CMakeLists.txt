cmake_minimum_required(VERSION 3.8)
project(scitos_msgs)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################
## Find  dependencies                         ##
################################################
## Find ament macros and libraries
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################
set(msg_files
  "msg/BarrierStatus.msg"
  "msg/BatteryState.msg"
  "msg/BumperStatus.msg"
  "msg/ChargerStatus.msg"
  "msg/DriveStatus.msg"
  "msg/EmergencyStopStatus.msg"
  "msg/MenuEntry.msg"
  "msg/Mileage.msg"
  "msg/RfidTag.msg"
)
set(srv_files
  "srv/ChangeForce.srv"
  "srv/EmergencyStop.srv"
  "srv/EnableMotors.srv"
  "srv/EnableRfid.srv"
  "srv/ResetBarrierStop.srv"
  "srv/ResetMotorStop.srv"
  "srv/ResetOdometry.srv"
  "srv/SavePersistentErrors.srv"
  "srv/SuspendBumper.srv"
)

rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  ${srv_files}
  DEPENDENCIES std_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## ament specific configuration ##
###################################
ament_export_dependencies(rosidl_default_runtime)

###########
## Build ##
###########

#############
## Install ##
#############

#############
## Testing ##
#############
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()