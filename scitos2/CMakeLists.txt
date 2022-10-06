cmake_minimum_required(VERSION 3.8)
project(scitos2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)

ament_package()