cmake_minimum_required(VERSION 2.8.3)
project(pcl_test)
SET(CMAKE_BUILD_TYPE "Release")
## Compile as C++11, supported in ROS Melodic and newer
add_compile_options(-std=c++11)
## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
 # pcl_ros
  roscpp
  sensor_msgs
  std_msgs
)

## System dependencies are found with CMake's conventions

catkin_package(
	#INCLUDE_DIRS include
#  LIBRARIES pcl_test
  #CATKIN_DEPENDS  pcl_ros roscpp sensor_msgs
#  DEPENDS system_lib
)
find_package(PCL 1.8 REQUIRED)

include_directories(
 include
  ${PCL_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
 add_executable(${PROJECT_NAME}_node src/pcl_test_node.cpp src/pcl_test_core.cpp src/pcl_test_core.h)


## Specify libraries to link a library or executable target against
 target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
   ${PCL_LIBRARIES})





