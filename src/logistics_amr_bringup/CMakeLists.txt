cmake_minimum_required(VERSION 3.8)
project(logistics_amr_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/slam_readiness_checker.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
