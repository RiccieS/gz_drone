cmake_minimum_required(VERSION 3.10)
project(drone_lidar_controller)

find_package(gz-transport13 REQUIRED)

add_executable(lidar_control lidar_control.cpp)
target_link_libraries(lidar_control gz-transport13::gz-transport13)