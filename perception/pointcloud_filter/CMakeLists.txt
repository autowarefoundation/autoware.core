cmake_minimum_required(VERSION 3.14)
project(autoware_ground_filter)

find_package(autoware_cmake REQUIRED)
autoware_package()

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

include_directories(
  include
  SYSTEM
    ${Boost_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${GRID_MAP_INCLUDE_DIR}
)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/scan_ground_filter/node.cpp
  src/scan_ground_filter/grid_ground_filter.cpp
  src/scan_ground_filter/sanity_check.cpp
)

target_link_libraries(${PROJECT_NAME}
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES}
)

# ========== Ground Filter ==========
# -- Scan Ground Filter --
rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "autoware::ground_filter::ScanGroundFilterComponent"
  EXECUTABLE scan_ground_filter_node)

# Check if the directory exists
if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/test/data")
    # Prepare .pcd file for test
    file(MAKE_DIRECTORY "${CMAKE_CURRENT_SOURCE_DIR}/test/data")
endif()

# Remove the file for idempotency
file(REMOVE "${CMAKE_CURRENT_SOURCE_DIR}/test/data/test.pcd")

# Download the file
file(DOWNLOAD
    # July 23, 2024
    "https://raw.githubusercontent.com/autowarefoundation/autoware_universe/5153fdd1739c2f95e4c0ecc17bcf4dce45ed46b8/perception/autoware_ground_segmentation/test/data/test.pcd"
    "${CMAKE_CURRENT_SOURCE_DIR}/test/data/test.pcd"
    EXPECTED_HASH SHA256=b433591e91d75d48911ab5e445f02034d93865a0e707fc6942494907c75dcda0
    TIMEOUT 60               # Total time allowed for download
    INACTIVITY_TIMEOUT 30    # Time in seconds with no data received
    STATUS status
    LOG log
)

list(GET status 0 status_code)
if(NOT status_code EQUAL 0)
    message(FATAL_ERROR "Download failed: ${log}")
endif()

ament_auto_package(INSTALL_TO_SHARE
  launch
  test/data
  config
)

# Resolve system dependency on yaml-cpp, which apparently does not
# provide a CMake find_package() module.
find_package(PkgConfig REQUIRED)
pkg_check_modules(YAML_CPP REQUIRED yaml-cpp)
find_path(YAML_CPP_INCLUDE_DIR
          NAMES yaml_cpp.h
          PATHS ${YAML_CPP_INCLUDE_DIRS})
find_library(YAML_CPP_LIBRARY
            NAMES YAML_CPP
            PATHS ${YAML_CPP_LIBRARY_DIRS})

link_directories(${YAML_CPP_LIBRARY_DIRS})
# cspell:ignore DHAVE, YAMLCPP
if(NOT ${YAML_CPP_VERSION} VERSION_LESS "0.5")
  add_definitions(-DHAVE_NEW_YAMLCPP)
endif()


if(BUILD_TESTING)
  find_package(ament_cmake_ros REQUIRED)

  ament_add_ros_isolated_gtest(test_scan_ground_filter
    test/test_scan_ground_filter.cpp
  )

  target_link_libraries(test_scan_ground_filter
    ${PROJECT_NAME}
    ${YAML_CPP_LIBRARIES}
  )

endif()
