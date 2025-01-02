cmake_minimum_required(VERSION 3.10)
project(drone_controller)

# Find required Gazebo and Ignition libraries
find_package(gazebo REQUIRED)
find_package(ignition-math6 REQUIRED)

# Include directories
include_directories(
  ${GAZEBO_INCLUDE_DIRS}
  ${IGNITION-MATH_INCLUDE_DIRS}
)

# Link directories
link_directories(
  ${GAZEBO_LIBRARY_DIRS}
  ${IGNITION-MATH_LIBRARY_DIRS}
)

# Add your plugin
add_library(drone_plugin SHARED scripts/DroneMovement.cpp scripts/pokus.cpp)

# Link your plugin with the required libraries
target_link_libraries(drone_plugin ${GAZEBO_LIBRARIES} ${IGNITION-MATH_LIBRARIES})

# Ensure the libraries are built correctly
add_definitions(${GAZEBO_CXX_FLAGS})
