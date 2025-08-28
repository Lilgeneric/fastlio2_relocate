cmake_minimum_required(VERSION 3.0.2)
project(pcl_downsample)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  pcl_ros
  roscpp
  sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pointcloud_downsample
#  CATKIN_DEPENDS pcl_conversions pcl_ros roscpp sensor_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(pcd_downsample_node src/pcd_downsample_node.cpp)

target_link_libraries(pcd_downsample_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
