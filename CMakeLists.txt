cmake_minimum_required(VERSION 3.3)
project(semantic_mesh_localization)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf
  pcl_conversions
  pcl_ros
  cv_bridge
  image_transport
)

find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(VTK REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
    ${VTK_USE_FILE}
    ${OpenCV_INCLUDE_DIRS}
)

catkin_package(
	CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs tf pcl_conversions
	DEPENDS EIGEN3 PCL
	INCLUDE_DIRS include
	LIBRARIES semloam
)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_subdirectory(src/lib)

add_executable(pointcloud_to_mesh src/pointcloud_to_mesh.cpp)
target_link_libraries(pointcloud_to_mesh ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${VTK_LIBRARIES} ${OpenCV_LIBRARIES} semloam)




#endif()
