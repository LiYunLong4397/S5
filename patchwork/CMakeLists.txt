cmake_minimum_required(VERSION 3.0.2)
project(patchwork)

add_compile_options(-std=c++17)
set(CMAKE_BUILD_TYPE "Release")

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        roslaunch
        cv_bridge
        pcl_conversions
        pcl_ros
        geometry_msgs
        laser_geometry
        sensor_msgs
        message_generation
        )

add_message_files(
        FILES
        node.msg
        ground_estimate.msg
)

generate_messages(
        DEPENDENCIES
        std_msgs
        geometry_msgs
        sensor_msgs
)

find_package(PCL 1.10 REQUIRED)
find_package(Boost 1.54 REQUIRED)

catkin_package(
        INCLUDE_DIRS
        LIBRARIES
        CATKIN_DEPENDS roscpp rospy std_msgs
)

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        include
)

add_executable(ros_kitti nodes/ros_kitti.cpp)
target_link_libraries(ros_kitti ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(ros_kitti patchwork_generate_messages_cpp)

add_executable(ros_kitti_publisher nodes/ros_kitti_publisher.cpp)
target_link_libraries(ros_kitti_publisher ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(ros_kitti_publisher patchwork_generate_messages_cpp)

add_executable(patch nodes/patch.cpp)
target_link_libraries(patch ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(patch patchwork_generate_messages_cpp)
