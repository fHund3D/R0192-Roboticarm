cmake_minimum_required(VERSION 3.8)
project(r0192_canbus)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Interfaces generieren (falls die srv hier liegt)
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/MotorCommand.srv"
  "msg/AxisState.msg"
)

include_directories(include)

# Das Programm (Executable) aus allen Dateien bauen
add_executable(can_service_node 
  src/can_service_node.cpp
  src/CanCommunication.cpp
  src/GDS68Driver.cpp
  src/GDS34Driver.cpp
)

target_include_directories(can_service_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Abhängigkeiten verknüpfen
ament_target_dependencies(can_service_node rclcpp std_msgs)
rosidl_get_typesupport_target(service_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")
target_link_libraries(can_service_node ${service_typesupport_target})

install(TARGETS
  can_service_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)

ament_package()