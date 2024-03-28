cmake_minimum_required(VERSION 3.8)
project(autoware_node)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

autoware_package()

ament_auto_add_library(${PROJECT_NAME} src/autoware_node.cpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_add_ros_isolated_gtest(test_autoware_node test/test_autoware_node.cpp)
  target_link_libraries(test_autoware_node ${PROJECT_NAME})
endif()

ament_auto_package()
