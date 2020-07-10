cmake_minimum_required(VERSION 2.8.3)
project(grasping_point)

add_compile_options(-std=c++11)

LIST( APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake" )
find_package(catkin REQUIRED COMPONENTS
  roscpp
)

find_package(Eigen3 REQUIRED)

catkin_package(
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS  
  roscpp 
  rospy  
  std_msgs  
  DEPENDS 
)
###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${RBDL_INCLUDE_DIRS}  
)
