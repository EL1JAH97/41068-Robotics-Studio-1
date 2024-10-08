cmake_minimum_required(VERSION 3.8)
project(sp3_slo3_6)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(gazebo_msgs REQUIRED)

add_executable(spawn_cylinder src/spawn_cylinder.cpp)
ament_target_dependencies(spawn_cylinder rclcpp gazebo_msgs)

add_executable(detect_cylinder src/detect_cylinder.cpp)
ament_target_dependencies(detect_cylinder rclcpp visualization_msgs sensor_msgs)

install(TARGETS
  detect_cylinder
  spawn_cylinder
  DESTINATION lib/${PROJECT_NAME})

ament_package()

