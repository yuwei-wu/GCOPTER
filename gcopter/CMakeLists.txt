cmake_minimum_required(VERSION 3.5)
project(gcopter)

# Use C++14
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_BUILD_TYPE Release)
add_compile_options(-Wall -O3 -fPIC)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(ompl REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${OMPL_INCLUDE_DIRS}
)

# Library
add_library(gcopter src/global_planning.cpp)

target_link_libraries(gcopter
  ${OMPL_LIBRARIES}
)

ament_target_dependencies(gcopter
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# Executable
add_executable(global_planning_node src/global_planning.cpp)

target_link_libraries(global_planning_node
  gcopter
)

ament_target_dependencies(global_planning_node
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
)

# Install targets
install(TARGETS
  gcopter
  global_planning_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include/
)

# Export package
ament_export_include_directories(include)
ament_export_libraries(gcopter)
ament_export_dependencies(
  rclcpp
  std_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  ompl
  Eigen3
)

ament_package()
