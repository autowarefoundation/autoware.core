cmake_minimum_required(VERSION 3.8)
project(autoware_control_center)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/control_center_node.cpp
  include/autoware/control_center/control_center_node.hpp
)

# In order to instantiate only a single instance, we have to use custom main function
#rclcpp_components_register_node(${PROJECT_NAME}
#  PLUGIN "autoware::control_center::ControlCenter"
#  EXECUTABLE ${PROJECT_NAME}_node
#)

ament_auto_add_executable(${PROJECT_NAME}_node
  src/control_center_main.cpp
)

if(BUILD_TESTING)
  add_smoke_test(${PROJECT_NAME} ${PROJECT_NAME}_node
    TEST_PARAM_FILENAMES "config/control_center.param.yaml"
  )

  install(DIRECTORY
    test/config/
    DESTINATION share/${PROJECT_NAME}/test/config/
  )

  file(GLOB_RECURSE TEST_FILES test/*.cpp)

  foreach(TEST_FILE ${TEST_FILES})
    # Get the test name without directory and extension
    get_filename_component(TEST_NAME ${TEST_FILE} NAME_WE)

    # Add each test separately
    ament_add_ros_isolated_gtest(${TEST_NAME} ${TEST_FILE} TIMEOUT 10)
    target_include_directories(${TEST_NAME} PRIVATE src/include)
    target_link_libraries(${TEST_NAME} ${PROJECT_NAME})
    ament_target_dependencies(${TEST_NAME}
      rclcpp
      rclcpp_lifecycle
      autoware_utils
      autoware_control_center_msgs)
  endforeach()
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  config)
