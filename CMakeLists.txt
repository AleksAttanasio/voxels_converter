cmake_minimum_required(VERSION 3.6)
project(boxels_converter)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(main main.cpp BoxelsConversion.cpp BoxelsConversion.h)
target_link_libraries (main ${PCL_LIBRARIES})