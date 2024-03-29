cmake_minimum_required(VERSION 2.8.3)
project(pcl_csf)

add_compile_options(-std=c++14)




find_package(catkin REQUIRED COMPONENTS
pcl_ros
roscpp
sensor_msgs
)

find_package(PCL 1.7 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp sensor_msgs pcl_ros
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(${PROJECT_NAME}_in_node  src/pcl_csf_core_in.cpp src/c2cdist.cpp src/Cloth.cpp src/Constraint.cpp src/CSF.cpp
 src/Particle.cpp src/point_cloud.cpp src/Rasterization.cpp src/XYZReader.cpp)

add_executable(${PROJECT_NAME}_out_node  src/pcl_csf_core_out.cpp src/c2cdist.cpp src/Cloth.cpp src/Constraint.cpp src/CSF.cpp
 src/Particle.cpp src/point_cloud.cpp src/Rasterization.cpp src/XYZReader.cpp)

add_executable(${PROJECT_NAME}_fast_node  src/pcl_csf_core_fast.cpp src/c2cdist.cpp src/Cloth.cpp src/Constraint.cpp src/CSF.cpp
 src/Particle.cpp src/point_cloud.cpp src/Rasterization.cpp src/XYZReader.cpp)


target_link_libraries(${PROJECT_NAME}_in_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_out_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_fast_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
