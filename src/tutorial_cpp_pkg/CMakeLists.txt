cmake_minimum_required(VERSION 3.8)
project(tutorial_cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)


set(msg_files
  "msg/TestTopic.msg"
  "srv/TestService.srv"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
)
ament_export_dependencies(rosidl_default_runtime)


rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} rosidl_typesupport_cpp)


add_executable(topic_publisher src/topic_publisher.cpp)
ament_target_dependencies(topic_publisher rclcpp)
target_link_libraries(topic_publisher "${cpp_typesupport_target}")


add_executable(topic_subscription src/topic_subscription.cpp)
ament_target_dependencies(topic_subscription rclcpp)
target_link_libraries(topic_subscription "${cpp_typesupport_target}")


add_executable(service_server src/service_server.cpp)
ament_target_dependencies(service_server rclcpp)
target_link_libraries(service_server "${cpp_typesupport_target}")

add_executable(service_client src/service_client.cpp)
ament_target_dependencies(service_client rclcpp)
target_link_libraries(service_client "${cpp_typesupport_target}")

add_executable(lifecycle_number_publisher src/lifecycle_number_publisher.cpp)
ament_target_dependencies(lifecycle_number_publisher rclcpp rclcpp_lifecycle example_interfaces)


install(TARGETS
  topic_publisher
  topic_subscription
  service_server
  service_client
  lifecycle_number_publisher
  DESTINATION lib/${PROJECT_NAME})

ament_package()
