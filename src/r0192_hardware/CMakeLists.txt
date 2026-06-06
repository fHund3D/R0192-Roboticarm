cmake_minimum_required(VERSION 3.8)
project(r0192_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(r0192_canbus REQUIRED)
find_package(std_srvs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(r0192_interfaces REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/r0192_hardware_interface.cpp
  src/homing_controller.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  hardware_interface
  pluginlib
  r0192_canbus
  std_srvs
  controller_manager_msgs
)

pluginlib_export_plugin_description_file(hardware_interface r0192_hardware_plugin.xml)

# Central robot state-machine node (standalone executable).
add_executable(robot_state_manager src/robot_state_manager.cpp)
ament_target_dependencies(robot_state_manager
  rclcpp
  std_srvs
  controller_manager_msgs
  r0192_interfaces
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS robot_state_manager
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(FILES r0192_hardware_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

ament_package()
