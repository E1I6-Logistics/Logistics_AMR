cmake_minimum_required(VERSION 3.8)
project(logitle_pose_cpp)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(logitle_pose_publisher_cpp
  src/logitle_pose_publisher_cpp.cpp
)
target_compile_features(logitle_pose_publisher_cpp PUBLIC cxx_std_17)

ament_target_dependencies(logitle_pose_publisher_cpp
  rclcpp
  geometry_msgs
  tf2
  tf2_ros
)

install(TARGETS logitle_pose_publisher_cpp
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
