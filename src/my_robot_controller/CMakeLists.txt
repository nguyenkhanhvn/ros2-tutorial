cmake_minimum_required(VERSION 3.8)
project(my_robot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(my_robot_interfaces REQUIRED)
find_package(example_interfaces REQUIRED)

add_executable(${PROJECT_NAME} src/main.cpp)
ament_target_dependencies(${PROJECT_NAME} rclcpp rclcpp_action my_robot_interfaces)

install(TARGETS
${PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
