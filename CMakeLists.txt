cmake_minimum_required(VERSION 3.1.0)
project(nuscenes_pcl_viz)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
# set(CMAKE_CXX_FLAGS         "${CMAKE_CXX_FLAGS} -Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG   "-O0 -g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -DNDEBUG")

find_package(PkgConfig REQUIRED)
pkg_check_modules(jsoncpp REQUIRED jsoncpp)
pkg_check_modules(yamlcpp REQUIRED yaml-cpp)

find_package(PCL REQUIRED COMPONENTS common visualization)

include_directories(
    include
)

add_executable(nuscenes_pcl_viz src/nuscenes_pcl_viz.cpp)
target_include_directories(nuscenes_pcl_viz PUBLIC ${PCL_INCLUDE_DIRS} ${jsoncpp_INCLUDE_DIRS} ${yamlcpp_INCLUDEDIR})
target_link_libraries(nuscenes_pcl_viz PUBLIC ${PCL_LIBRARIES} ${jsoncpp_LIBRARIES} ${yamlcpp_LIBRARIES})

