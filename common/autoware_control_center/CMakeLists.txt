cmake_minimum_required(VERSION 3.8)
project(autoware_control_center)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

autoware_package()

ament_auto_add_executable(${PROJECT_NAME} src/autoware_control_center_node.cpp
  src/autoware_control_center.cpp
  src/node_registry.cpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(rclcpp REQUIRED)
  find_package(autoware_utils REQUIRED)
  ament_lint_auto_find_test_dependencies()
  ament_add_gtest(test_autoware_control_center
                  test/test_autoware_control_center.cpp
                  test/test_node_registry.cpp
                  src/autoware_control_center.cpp
                  src/node_registry.cpp)
  target_include_directories(test_autoware_control_center PRIVATE ${PROJECT_SOURCE_DIR}/include)
  ament_target_dependencies(test_autoware_control_center
                          "rclcpp"
                          "autoware_utils"
                          "rclcpp_lifecycle"
                          "autoware_control_center_msgs")
endif()

ament_auto_package()
