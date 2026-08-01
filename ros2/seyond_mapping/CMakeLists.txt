cmake_minimum_required(VERSION 3.16)
project(seyond_mapping)

if(NOT CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64|x86_64|AMD64)$")
  message(WARNING "This package is tested on aarch64 and x86_64; detected ${CMAKE_SYSTEM_PROCESSOR}")
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# Keep the ROS integration separate from the original SDK targets. In the
# mtr-boat-core monorepo, this package is under ros2/seyond_mapping while
# vcstool imports the SDK beside mtr-boat-core under workspace/src.
get_filename_component(
  DEFAULT_SDK_ROOT
  "${CMAKE_CURRENT_SOURCE_DIR}/../../../inno-lidar-sdk"
  ABSOLUTE)
set(SDK_ROOT "${DEFAULT_SDK_ROOT}" CACHE PATH "Root of the built inno-lidar-sdk tree")
foreach(SDK_LIB innolidarsdkclient innolidarsdkcommon innolidarutils)
  if(NOT EXISTS "${SDK_ROOT}/lib/lib${SDK_LIB}.a")
    message(FATAL_ERROR
      "Missing ${SDK_ROOT}/lib/lib${SDK_LIB}.a. Build inno-lidar-sdk first, "
      "import it with the workspace dependencies, or pass "
      "-DSDK_ROOT=/absolute/sdk/path.")
  endif()
  add_library(${SDK_LIB} STATIC IMPORTED)
  set_target_properties(${SDK_LIB} PROPERTIES IMPORTED_LOCATION "${SDK_ROOT}/lib/lib${SDK_LIB}.a")
endforeach()

add_executable(seyond_pointcloud_node src/seyond_pointcloud_node.cpp)
target_include_directories(seyond_pointcloud_node SYSTEM PRIVATE
  "${SDK_ROOT}"
  "${SDK_ROOT}/src")
target_link_libraries(seyond_pointcloud_node
  innolidarsdkclient innolidarsdkcommon innolidarutils pthread)
ament_target_dependencies(seyond_pointcloud_node rclcpp sensor_msgs)

add_executable(map_accumulator_node src/map_accumulator_node.cpp)
ament_target_dependencies(map_accumulator_node
  geometry_msgs nav_msgs rclcpp sensor_msgs std_srvs)

add_executable(live_web_viewer_node src/live_web_viewer_node.cpp)
target_link_libraries(live_web_viewer_node Boost::system pthread)
ament_target_dependencies(live_web_viewer_node rclcpp sensor_msgs)

install(TARGETS seyond_pointcloud_node map_accumulator_node live_web_viewer_node
  DESTINATION lib/${PROJECT_NAME})
install(
  DIRECTORY config launch rviz web
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE)

ament_package()
