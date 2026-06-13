cmake_minimum_required(VERSION 3.8)
project(r0192_program_executor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(r0192_interfaces REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

add_executable(program_executor
  src/program_executor_node.cpp
  src/program_loader.cpp
)
target_include_directories(program_executor PRIVATE include)
ament_target_dependencies(program_executor
  rclcpp
  rclcpp_action
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  tf2_ros
  visualization_msgs
  moveit_ros_planning_interface
  moveit_core
  moveit_msgs
  r0192_interfaces
  yaml_cpp_vendor
)
target_link_libraries(program_executor yaml-cpp::yaml-cpp)

install(TARGETS program_executor
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  # Unit tests for the YAML loader/validator (the single parse point). The
  # loader source is compiled into the test directly (no link to the node).
  ament_add_gtest(test_program_loader
    test/test_program_loader.cpp
    src/program_loader.cpp
  )
  target_include_directories(test_program_loader PRIVATE include)
  ament_target_dependencies(test_program_loader geometry_msgs yaml_cpp_vendor)
  target_link_libraries(test_program_loader yaml-cpp::yaml-cpp)
endif()

ament_package()
