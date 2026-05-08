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


include_directories(include)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)

ament_package()