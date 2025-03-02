cmake_minimum_required(VERSION 3.8)
project(autoware_lanelet2_utility)

find_package(autoware_cmake REQUIRED)
autoware_package()

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/kind.cpp
)

if(BUILD_TESTING)
  find_package(ament_cmake_ros REQUIRED)
  ament_auto_find_test_dependencies()

  file(GLOB_RECURSE test_files test/*.cpp)

  foreach (test_file IN LISTS test_files)
    get_filename_component(test_file_name ${test_file} NAME)
    ament_auto_add_gtest(${test_file_name}_${PROJECT_NAME} ${test_file})
    target_link_libraries(${test_file_name}_${PROJECT_NAME}
      ${PROJECT_NAME}
    )
  endforeach()
endif()

ament_auto_package(INSTALL_TO_SHARE
  sample_map
)

install(PROGRAMS
  scripts/lanelet_anonymizer.py
  DESTINATION lib/${PROJECT_NAME}
)
