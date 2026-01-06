cmake_minimum_required(VERSION 3.1.0)
project(m_detector)

SET(CMAKE_BUILD_TYPE "Release")

ADD_COMPILE_OPTIONS(-std=c++17 )
set( CMAKE_CXX_FLAGS "-std=c++17 -O3 -ltbb" ) 

add_definitions(-DROOT_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fexceptions" )
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -ltbb -pthread -std=c++1z -std=c++17 -fexceptions")

message("Current CPU archtecture: ${CMAKE_SYSTEM_PROCESSOR}")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "(x86)|(X86)|(amd64)|(AMD64)" )
  include(ProcessorCount)
  ProcessorCount(N)
  message("Processer number:  ${N}")
  # if(N GREATER 15)
  #   add_definitions(-DMP_EN)
  #   add_definitions(-DMP_PROC_NUM=14)
  #   message("core for MP:  4")
  # elseif(N GREATER 5)
  if(N GREATER 5)
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM=4)
    message("core for MP:  3")
  elseif(N GREATER 3)
    math(EXPR PROC_NUM "${N} - 2")
    add_definitions(-DMP_EN)
    add_definitions(-DMP_PROC_NUM="${PROC_NUM}")
    message("core for MP:  ${PROC_NUM}")
  else()
    add_definitions(-DMP_PROC_NUM=1)
  endif()
else()
  add_definitions(-DMP_PROC_NUM=1)
endif()

find_package(OpenMP QUIET)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}   ${OpenMP_C_FLAGS}")

find_package(PythonLibs REQUIRED)
# find_path(MATPLOTLIB_CPP_INCLUDE_DIRS "matplotlibcpp.h")

#[[ 
set(TBB_DIR "/usr/local/tbb-2019_U8")
include("/home/zyh/add/oneTBB-2019_U8/cmake/TBBBuild.cmake")
tbb_build(TBB_ROOT "/home/zyh/add/oneTBB-2019_U8" CONFIG_DIR TBB_DIR) 
]]
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  pcl_ros
  tf
  livox_ros_driver
  message_generation
  eigen_conversions
)



find_package(Eigen3 REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(TBB REQUIRED)
find_package(OpenCV REQUIRED)

message(Eigen: ${EIGEN3_INCLUDE_DIR})

include_directories(
	${catkin_INCLUDE_DIRS} 
  ${EIGEN3_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
  ${PYTHON_INCLUDE_DIRS}
  include)

generate_messages(
 DEPENDENCIES
 geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs message_runtime
  DEPENDS EIGEN3 PCL
  INCLUDE_DIRS
)


add_executable(dynfilter src/dynfilter_with_odom.cpp src/DynObjFilter.cpp src/DynObjCluster.cpp 
                          include/lidar_perception/min_box/min_box.cc
                          include/lidar_perception/object.cc
                          include/lidar_perception/hm_tracker/hm_tracker.cc
                          include/lidar_perception/hm_tracker/feature_descriptor.cc
                          include/lidar_perception/hm_tracker/hungarian_matcher.cc
                          include/lidar_perception/hm_tracker/kalman_filter.cc
                          include/lidar_perception/hm_tracker/object_track.cc
                          include/lidar_perception/hm_tracker/track_object_distance.cc
                          include/lidar_perception/hm_tracker/tracked_object.cc
                          include/lidar_perception/hungarian/hungarian_bigraph_matcher.cc
                          include/lidar_perception/common/graph_util.cc
                          include/lidar_perception/common/geometry_util.cc
                          src/pcl_clip/pcl_clip.cpp
)
target_link_libraries(dynfilter ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PYTHON_LIBRARIES} ${OpenCV_LIBS} TBB::tbb)
target_include_directories(dynfilter PRIVATE ${PYTHON_INCLUDE_DIRS})



add_executable(imu2odom src/imu2odom.cpp)
target_link_libraries(imu2odom ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PYTHON_LIBRARIES} ${OpenCV_LIBS} TBB::tbb)
target_include_directories(imu2odom PRIVATE ${PYTHON_INCLUDE_DIRS})

add_executable(evaluate_odom src/evaluate_odom.cpp)
target_link_libraries(evaluate_odom ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${PYTHON_LIBRARIES} ${OpenCV_LIBS} TBB::tbb)
target_include_directories(evaluate_odom PRIVATE ${PYTHON_INCLUDE_DIRS})


