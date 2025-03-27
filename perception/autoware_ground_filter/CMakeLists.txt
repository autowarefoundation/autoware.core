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
  src/node.cpp
  src/grid_ground_filter.cpp
  src/sanity_check.cpp
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

ament_auto_package(INSTALL_TO_SHARE
  launch
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
