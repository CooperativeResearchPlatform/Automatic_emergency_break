cmake_minimum_required(VERSION 3.8)
project(basic_concepts)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)


include_directories(include)

add_executable(publisher_demo "src/publisherDemo.cpp")
ament_target_dependencies(publisher_demo rclcpp std_msgs)

add_executable(subscriber_demo "src/subscriberDemo.cpp")
ament_target_dependencies(subscriber_demo rclcpp std_msgs)

install(TARGETS
  publisher_demo
  subscriber_demo
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
