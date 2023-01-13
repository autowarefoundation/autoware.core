cmake_minimum_required(VERSION 3.14)
project(autoware_control_center_msgs)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(msg_dependencies
  unique_identifier_msgs)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/StatusRegistration.msg"
  "srv/AutowareNodeRegister.srv"
  DEPENDENCIES ${msg_dependencies}
  ADD_LINTER_TESTS)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
