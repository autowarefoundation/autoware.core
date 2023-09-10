cmake_minimum_required(VERSION 3.8)
project(autoware_control_center)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

autoware_package()

ament_auto_add_executable(${PROJECT_NAME} src/autoware_control_center_node.cpp
  src/autoware_control_center.cpp
  src/node_registry.cpp)
#target_link_libraries(${PROJECT_NAME})

#if (BUILD_TESTING)
#    find_package(ament_lint_auto REQUIRED)
#    ament_lint_auto_find_test_dependencies()
#    ament_add_ros_isolated_gtest(test_autoware_control_center test/test_autoware_control_center.cpp)
#    target_link_libraries(test_autoware_control_center ${PROJECT_NAME})
#endif ()

ament_auto_package()
