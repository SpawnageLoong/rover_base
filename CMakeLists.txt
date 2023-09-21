cmake_minimum_required(VERSION 3.8)
project(rover_base)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rover_interfaces REQUIRED)

add_executable(rover_base src/rover_base.cpp)
ament_target_dependencies(rover_base rclcpp geometry_msgs rover_interfaces)

install(TARGETS
  rover_base
  DESTINATION lib/${PROJECT_NAME})

ament_package()
