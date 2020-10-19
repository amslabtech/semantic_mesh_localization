add_library(semloam
	pointcloudtomesh.cpp)

target_link_libraries(semloam ${catkin_LIBRARIES} ${PCL_LIBRARIES} )
