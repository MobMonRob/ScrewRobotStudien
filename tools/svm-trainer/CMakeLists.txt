cmake_minimum_required(VERSION 3.1 FATAL_ERROR)

project(svm-trainer)

find_package(PCL 1.8 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(svm-trainer main.cpp)
target_link_libraries(svm-trainer ${PCL_LIBRARIES})