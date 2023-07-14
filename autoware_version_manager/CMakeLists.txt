cmake_minimum_required(VERSION 3.14)
project(autoware_version_manager)

find_package(autoware_cmake REQUIRED)
autoware_package()

set(NODE_NAME ${PROJECT_NAME}_node)
set(EXEC_NAME ${PROJECT_NAME}_exe)

ament_auto_add_library(${NODE_NAME}
  src/include/autoware_version_manager_core.hpp
  src/include/parse_version.hpp
  src/include/version_types.hpp
  src/autoware_version_manager_core.cpp
  src/parse_version.cpp)

rclcpp_components_register_node(${NODE_NAME}
  PLUGIN "autoware_version_manager::AutowareVersionManagerNode"
  EXECUTABLE ${EXEC_NAME})

ament_auto_package(INSTALL_TO_SHARE
  launch
  data)
