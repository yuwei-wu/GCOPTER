cmake_minimum_required(VERSION 3.10)

project(gcopter)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -fPIC")

find_package(Eigen3 REQUIRED)
find_package(ompl REQUIRED)
find_package(yaml-cpp REQUIRED)

include_directories(
    include
    ${OMPL_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    ${YAML_CPP_INCLUDE_DIRS}
)

## Library
add_library(gcopter_lib
    src/global_planning.cpp
)

target_link_libraries(gcopter_lib
    ${OMPL_LIBRARIES}
    Eigen3::Eigen
)

target_include_directories(gcopter_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

## Example Executable
add_executable(global_planning_demo src/global_planning.cpp)

target_link_libraries(global_planning_demo
    gcopter_lib
    ${OMPL_LIBRARIES}
    Eigen3::Eigen
    yaml-cpp
)
