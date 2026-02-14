cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(voxel_filtering)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (voxel_filtering voxel_filtering.cpp)
target_link_libraries (voxel_filtering ${PCL_LIBRARIES})
