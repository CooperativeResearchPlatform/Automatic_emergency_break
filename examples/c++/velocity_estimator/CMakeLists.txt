cmake_minimum_required(VERSION 3.8)
project(velocity_estimator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

include_directories(include)

add_executable(pose_publisher src/posePublisher.cpp)
ament_target_dependencies(pose_publisher rclcpp geometry_msgs)

add_executable(velocity_estimator src/velocityEstimator.cpp)
ament_target_dependencies(velocity_estimator rclcpp geometry_msgs)


install(TARGETS
  pose_publisher
  velocity_estimator
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
