cmake_minimum_required(VERSION 3.8)
project(nav2_graph_waypoint_bt)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_util REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_ros REQUIRED)

add_library(${PROJECT_NAME} SHARED src/graph_waypoint_bt.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME} PRIVATE include)
ament_target_dependencies(${PROJECT_NAME}
  behaviortree_cpp geometry_msgs nav2_util rclcpp tf2_ros)
target_compile_definitions(${PROJECT_NAME} PRIVATE BT_PLUGIN_EXPORT)

add_executable(plugin_smoke_test src/plugin_smoke_test.cpp)
target_compile_features(plugin_smoke_test PUBLIC cxx_std_17)
ament_target_dependencies(plugin_smoke_test behaviortree_cpp)

install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)
install(TARGETS plugin_smoke_test RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY behavior_trees/ DESTINATION share/${PROJECT_NAME}/behavior_trees)
ament_package()
