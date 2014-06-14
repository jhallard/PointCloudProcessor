cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
 
project(PointCloudProcessing)
 
find_package(PCL 1.7 REQUIRED)
 
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
 
set(PCL_BUILD_TYPE Release)
 
file(GLOB PointCloudGrabber.cpp
    "src/*.h"
    "src/*.cpp"
)
  
file(GLOB CloudGrabber_SRC
    "src/*.h"
    "src/*.cpp"
)
find_package( catkin REQUIRED COMPONENTS roscpp )

add_executable(PointCloudProcessor ${PointCloudGrabber.cpp})
 
target_link_libraries (PointCloudProcessor ${PCL_LIBRARIES})

target_link_libraries (PointCloudProcessor ${catkin_LIBRARIES})
