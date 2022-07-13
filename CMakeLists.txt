cmake_minimum_required(VERSION 3.0.2)
project(occupancy_segmentation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  sensor_msgs
  tf
  tf2
  visualization_msgs
  pcl_ros
  roscpp
)

find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${Eigen_INCLUDE_DIRS})

link_directories(${PCL_LIBRARY_DIRS})

add_definitions(${PCL_DEFINITIONS})

catkin_package(
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}_node src/occupancy_node.cpp)
add_executable(${PROJECT_NAME}_imu_stab src/imu_stab.cpp)
add_executable(${PROJECT_NAME}_imu_fuse src/imu_fuse.cpp)
add_executable(${PROJECT_NAME}_imu_orientation src/imu_orientation.cpp)

target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_imu_stab ${catkin_LIBRARIES} ${Eigen_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_imu_fuse ${catkin_LIBRARIES})
target_link_libraries(${PROJECT_NAME}_imu_orientation ${catkin_LIBRARIES})
