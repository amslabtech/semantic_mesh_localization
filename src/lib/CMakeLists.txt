option(TF_AVAIL "Tensorflow available" OFF)
option(TRT_AVAIL "TensorRT available" OFF)
option(CUDA_AVAIL "CUDA available" OFF)

 ## Find libraries
# try the shared tensorflow
find_package(TensorflowCC COMPONENTS Shared)
if(TensorflowCC_Shared_FOUND)
    get_property(TF_LIBS TARGET TensorflowCC::Shared PROPERTY INTERFACE_LINK_LIBRARIES)
    get_property(TF_INCLUDE_DIRS TARGET TensorflowCC::Shared PROPERTY INTERFACE_INCLUDE_DIRECTORIES)
    add_definitions(-DTF_AVAIL)
    set(TF_AVAIL ON)
else()
    set(TF_AVAIL OFF)
endif()
  
# try to find CUDA
find_package(CUDA REQUIRED)
if (CUDA_FOUND)
    set(CUDA_AVAIL ON)
else()
    set(CUDA_AVAIL OFF)
endif (CUDA_FOUND)

# try to find the tensorRT modules
find_library(NVINFER NAMES nvinfer)
find_library(NVPARSERS NAMES nvparsers)
if(NVINFER AND NVPARSERS)
    set(TRT_AVAIL ON)
    add_definitions(-DTRT_AVAIL)
else()
    set(TRT_AVAIL OFF)
endif()

set(bonnet_core_LIBRARIES ${OpenCV_LIBS} ${YAML_CPP_LIBRARIES})
set(bonnet_under_SOURCES net.cpp bonnet.cpp bonnet_handler.cpp)

if(TF_AVAIL)
    list(APPEND bonnet_under_SOURCES netTF.cpp)
    list(APPEND bonnet_core_LIBRARIES ${TF_LIBS} )
endif()

if(TRT_AVAIL AND CUDA_AVAIL)
    list(APPEND bonnet_under_SOURCES netTRT.cpp)
    list(APPEND bonnet_core_LIBRARIES ${CUDA_LIBRARIES} nvinfer nvparsers)
endif()

add_library(
    semlocali
    pointcloudtomesh.cpp
    meshlocalization.cpp
    pointlocalization.cpp
    verifycallikelihood.cpp

    bonnet
    ${bonnet_under_SOURCES}
    )

target_link_libraries(
    semlocali
    ${catkin_LIBRARIES}
    ${PCL_LIBRARIES}
    ${OpenCV_LIBRARIES}
    
    #    bonnet
    ${bonnet_core_LIBRARIES}
    )



