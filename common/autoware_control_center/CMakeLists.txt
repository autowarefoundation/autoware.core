cmake_minimum_required(VERSION 3.8)
project(autoware_control_center)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/control_center_node.cpp
  src/node_registry.cpp
  src/include/control_center_node.hpp
  src/include/node_registry.hpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::control_center::ControlCenter"
  EXECUTABLE ${PROJECT_NAME}_node
)

if(BUILD_TESTING)
  file(GLOB_RECURSE TEST_SOURCES test/*.cpp)
  ament_add_ros_isolated_gtest(test_autoware_control_center ${TEST_SOURCES})
  target_include_directories(test_autoware_control_center PRIVATE src/include)
  target_link_libraries(test_autoware_control_center ${PROJECT_NAME})
  ament_target_dependencies(test_autoware_control_center
    rclcpp
    rclcpp_lifecycle
    autoware_utils
    autoware_control_center_msgs)
endif()

ament_auto_package()
