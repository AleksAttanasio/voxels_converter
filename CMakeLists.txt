cmake_minimum_required(VERSION 3.6)
project(voxels_converter)

find_package(PCL 1.2 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(main main.cpp VoxelsConversion.cpp VoxelsConversion.h)
target_link_libraries (main ${PCL_LIBRARIES})

add_executable(voxel_test voxel_test.cpp)
target_link_libraries (voxel_test ${PCL_LIBRARIES})

add_executable(pcd_reconstruction pcd_reconstruction.cpp VoxelsConversion.cpp VoxelsConversion.h GraspingDetection.cpp GraspingDetection.h)
target_link_libraries (pcd_reconstruction ${PCL_LIBRARIES})