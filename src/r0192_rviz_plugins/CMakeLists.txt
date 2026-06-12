cmake_minimum_required(VERSION 3.8)
project(r0192_rviz_plugins)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rviz_common REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(r0192_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)

# RViz panels are Qt widgets → let CMake run the meta-object compiler.
set(CMAKE_AUTOMOC ON)

add_library(${PROJECT_NAME} SHARED
  src/operator_panel.cpp
  src/jog_panel.cpp
  src/program_panel.cpp
  include/r0192_rviz_plugins/operator_panel.hpp
  include/r0192_rviz_plugins/jog_panel.hpp
  include/r0192_rviz_plugins/program_panel.hpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_action
  rviz_common
  pluginlib
  std_msgs
  std_srvs
  control_msgs
  geometry_msgs
  moveit_msgs
  sensor_msgs
  r0192_interfaces
  tf2
  tf2_ros
)
target_link_libraries(${PROJECT_NAME} Qt5::Widgets)

# Register the panel with pluginlib under the rviz_common package so RViz finds
# it via "Panels -> Add New Panel".
pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/
  DESTINATION include
)
install(FILES plugin_description.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_targets(export_${PROJECT_NAME})
ament_package()
