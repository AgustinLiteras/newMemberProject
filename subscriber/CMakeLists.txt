cmake_minimum_required(VERSION 3.8)
project(subscriber)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cipher_interfaces REQUIRED)       

add_executable(listener src/subscriber_member_function.cpp)
ament_target_dependencies(listener rclcpp cipher_interfaces)

install(TARGETS
  listener
  DESTINATION lib/${PROJECT_NAME})

ament_package()
