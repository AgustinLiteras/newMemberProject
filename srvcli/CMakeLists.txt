cmake_minimum_required(VERSION 3.8)
project(srvcli)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cipher_interfaces REQUIRED) 

add_executable(server src/server.cpp)
ament_target_dependencies(server rclcpp cipher_interfaces)

add_executable(talker src/publisher_member_function.cpp)
ament_target_dependencies(talker rclcpp cipher_interfaces)

install(TARGETS
    server
    talker
  DESTINATION lib/${PROJECT_NAME})

ament_package()
