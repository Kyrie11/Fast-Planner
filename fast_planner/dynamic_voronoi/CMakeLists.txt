cmake_minimum_required(VERSION 2.8.3)
project(dynamic_voronoi)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    geometry_msgs    
    )

#add_library(HYAS ${SOURCES} ${HEADERS})
catkin_package(
  INCLUDE_DIRS include
  LIBRARIES dynamic_voronoi
  CATKIN_DEPENDS)

add_library( dynamic_voronoi
             src/dynamicvoronoi.cpp
	     src/bucketedqueue.cpp 
	    )
target_link_libraries( dynamic_voronoi
    ${catkin_LIBRARIES} 
    )  

find_package(Eigen3 REQUIRED)


include_directories(
  include 
  SYSTEM 
  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  )



