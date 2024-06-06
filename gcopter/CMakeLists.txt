cmake_minimum_required(VERSION 3.10)

project(gcopter)

set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -fPIC")

find_package(Eigen3 REQUIRED)
find_package(ompl REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  sensor_msgs
  visualization_msgs
  kr_planning_msgs
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${OMPL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    include
)


catkin_package(
 INCLUDE_DIRS include
 DEPENDS OMPL
 LIBRARIES gcopter
)

add_executable(global_planning src/global_planning.cpp)

target_link_libraries(global_planning
  ${OMPL_LIBRARIES}
  ${catkin_LIBRARIES}
)


add_library(gcopter
  src/global_planning.cpp
  )
target_link_libraries(gcopter
  ${OMPL_LIBRARIES}
  ${catkin_LIBRARIES}
  )
add_dependencies(gcopter
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  )
