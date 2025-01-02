cmake_minimum_required(VERSION 3.10)
project(drone_movement)

# Find Gazebo transport library
find_package(gz-transport13 REQUIRED)

# Add executable for your program
add_executable(drone_movement scripts/pokus.cpp)

# Link your program with Gazebo transport
target_link_libraries(drone_movement gz-transport13::gz-transport13)
