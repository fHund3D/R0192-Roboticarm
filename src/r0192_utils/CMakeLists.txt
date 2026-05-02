cmake_minimum_required(VERSION 3.8)
project(r0192_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(r0192_msgs REQUIRED)
find_package(tf2 REQUIRED)

add_executable(angle_conversion src/angle_conversion.cpp)
ament_target_dependencies(angle_conversion rclcpp r0192_msgs tf2)

ament_package()