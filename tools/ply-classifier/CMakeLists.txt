cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

project(ply-classifier)

find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(ply-classifier main.cpp)
target_link_libraries(ply-classifier ${PCL_LIBRARIES})