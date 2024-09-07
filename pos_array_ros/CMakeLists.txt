cmake_minimum_required(VERSION 3.5)
project(pos_array_ros)

# Default to C++17
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pos_array_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# Declare the executable
add_executable(pos_array_publisher src/pos_array_publisher.cpp)

# Specify libraries to link the executable against
ament_target_dependencies(pos_array_publisher
  rclcpp
  pos_array_msgs
  geometry_msgs
)


add_executable(set_position_server src/set_position_server.cpp)
ament_target_dependencies(set_position_server
  rclcpp
  pos_array_msgs
)

add_executable(set_position_client src/set_position_client.cpp)
ament_target_dependencies(set_position_client
  rclcpp
  pos_array_msgs
)


# Install the executable
install(TARGETS
  pos_array_publisher
  set_position_server
  set_position_client
  DESTINATION lib/${PROJECT_NAME})

# Macro to register the package with ament
ament_package()
