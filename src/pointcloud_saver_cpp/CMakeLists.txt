cmake_minimum_required(VERSION 3.0.2)
project(pointcloud_saver_cpp)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  pcl_conversions
  pcl_ros
)

find_package(PCL REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp sensor_msgs pcl_conversions pcl_ros
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

add_executable(full_map_saver src/full_map_saver.cpp)
target_link_libraries(full_map_saver
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

install(TARGETS full_map_saver
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

