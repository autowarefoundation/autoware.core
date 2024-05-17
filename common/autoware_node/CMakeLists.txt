cmake_minimum_required(VERSION 3.8)
project(autoware_node)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} src/node.cpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_add_ros_isolated_gtest(test_autoware_node test/test_node.cpp)
  target_link_libraries(test_autoware_node ${PROJECT_NAME})
endif()

ament_auto_package()
