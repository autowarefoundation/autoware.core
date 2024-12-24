cmake_minimum_required(VERSION 3.8)
project(autoware_simple_pure_pursuit)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME}_lib SHARED
  DIRECTORY src
)

rclcpp_components_register_node(${PROJECT_NAME}_lib
  PLUGIN "autoware::control::simple_pure_pursuit::SimplePurePursuitNode"
  EXECUTABLE ${PROJECT_NAME}_exe
)

ament_auto_package(
  INSTALL_TO_SHARE
    # launch
    config
)
