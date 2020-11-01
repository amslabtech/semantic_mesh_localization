add_library(
    semloam
    bonnet
	pointcloudtomesh.cpp
    bonnet_handler.cpp
    bonnet.cpp
    net.cpp
    netTF.cpp
    netTRT.cpp
    )

target_link_libraries(
    semloam
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${bonnet_node_LIBRARIES}
    ${bonnet_core_LIBRARIES}
    ${OpenCV_LIBS}
    tensorflow_cc
    )


