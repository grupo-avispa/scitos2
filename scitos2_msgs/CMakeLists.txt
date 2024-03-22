cmake_minimum_required(VERSION 3.5)
project(scitos2_msgs)

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

###########
## Build ##
###########

#############
## Install ##
#############

# ############
# # Testing ##
# ############
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

###################################
## ament specific configuration ##
###################################
ament_export_dependencies(rosidl_default_runtime)
ament_package()