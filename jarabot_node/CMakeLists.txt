cmake_minimum_required(VERSION 3.8)
project(jarabot_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(jarabot_interfaces REQUIRED)

# find_package(rosidl_default_generators REQUIRED)

# rosidl_generate_interfaces(${PROJECT_NAME}
# "msg/Cmd.msg"
# "msg/Ecd.msg"
# DEPENDENCIES std_msgs
# )

include_directories(include)


add_executable(jara_controller src/jara_controller.cpp)
ament_target_dependencies(jara_controller rclcpp jarabot_interfaces geometry_msgs sensor_msgs)

add_executable(jara_driver src/jara_driver.cpp)
ament_target_dependencies(jara_driver rclcpp jarabot_interfaces)

add_executable(jara_odometry src/jara_odometry.cpp)
ament_target_dependencies(jara_odometry rclcpp jarabot_interfaces geometry_msgs tf2 tf2_ros nav_msgs tf2_geometry_msgs)


install(TARGETS
  jara_controller
  jara_driver
  jara_odometry
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME})


ament_package()
