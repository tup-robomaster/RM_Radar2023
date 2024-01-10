cmake_minimum_required(VERSION 3.9.0)
SET(CMAKE_BUILD_TYPE "Release")
project(radar2023 LANGUAGES CXX CUDA)
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  roslib
  std_msgs
  sensor_msgs
  cv_bridge
  message_generation
  message_runtime
  image_transport
)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(CUDA 11 REQUIRED)
find_package(fmt REQUIRED)
find_package(spdlog REQUIRED)
message("CUDA_LIBRARIES:${CUDA_LIBRARIES}")
message("CUDA_INCLUDE_DIRS:${CUDA_INCLUDE_DIRS}")
include_directories()
add_compile_options(-std=c++17)
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-z,stack-size=16000000")
set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -pthread -lpthread -O3  -Wall -fstack-protector-all")
set(CUDA_GEN_CODE "-gencode=arch=compute_86,code=sm_86")
set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS}  -std=c++17 -O3 -Xcompiler -fPIC -g -w ${CUDA_GEN_CODE}")

add_message_files(
  FILES
  Location.msg
  Locations.msg
)

generate_messages(
 DEPENDENCIES
 geometry_msgs
)

catkin_package(CATKIN_DEPENDS
	roscpp
  rospy
  roslib
  std_msgs
	sensor_msgs
  cv_bridge
  message_generation
  message_runtime
  image_transport
	)
# cuda
include_directories(/user/local/cuda/include)
link_directories(/user/local/cuda/lib)
# cudnn
include_directories(/user/local/cuda/cudnn/v8.0.4/include)
link_directories(/user/local/cuda/cudnn/v8.0.4/lib64)
# Tensorrt
include_directories(/usr/local/TensorRT/include)
link_directories(/usr/local/TensorRT/lib)
# mkl
include_directories(${MKLROOT}/include)
link_directories(${MKLROOT}/lib/intel64)
# fmt
include_directories(${FMT_INCLUDE_DIRS})

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
)
include_directories(${EIGEN3_INCLUDE_DIRS})
link_directories(
  ${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set(radar2023_source
  main.cpp
  RadarClass/TRTInference/TRTInfer/CUDA/preprocess.cu
  RadarClass/TRTInference/TRTInfer/src/Inference.cpp
  RadarClass/TRTGenerator/TRTGenerator/src/Generator.cpp
  RadarClass/Camera/src/camera.cpp
  RadarClass/Camera/src/VideoRecorder.cpp
  RadarClass/Common/src/algorithm.cpp
  RadarClass/Detectors/src/ArmorDetector.cpp
  RadarClass/Detectors/src/CarDetector.cpp
  RadarClass/Detectors/src/depthProcesser.cpp
  RadarClass/Detectors/src/MovementDetector.cpp
  RadarClass/Location/src/location.cpp
  RadarClass/Location/src/MapMapping.cpp
  RadarClass/Radar/src/Radar.cpp
  RadarClass/UART/src/offical_judge_Handler.cpp
  RadarClass/UART/src/serial.cpp
  RadarClass/UART/src/UART.cpp
  RadarClass/UART/src/UARTPasser.cpp
  RadarClass/Logger/src/Logger.cpp
  RadarClass/DsTracker/src/deepsort.cpp
  RadarClass/DsTracker/src/featuretensor.cpp
  RadarClass/DsTracker/src/kalmanfilter.cpp
  RadarClass/DsTracker/src/linear_assignment.cpp
  RadarClass/DsTracker/src/sort.cpp
  RadarClass/DsTracker/src/track.cpp
  RadarClass/DsTracker/src/tracker.cpp
  RadarClass/DsTracker/src/hungarianoper.cpp
  RadarClass/DsTracker/src/DsTracker.cpp
  RadarClass/Logger/src/ExpLog.cpp
  )

cuda_add_executable(radar2023
  ${radar2023_source}
)

add_executable(offline_point_cloud_publisher tools/offline_point_cloud_pub.cpp)
target_link_libraries(offline_point_cloud_publisher ${PCL_LIBRARIES}  ${catkin_LIBRARIES} )

add_executable(gui_node GUI/GUI.cpp)
target_link_libraries(gui_node ${catkin_LIBRARIES} )

target_sources(radar2023
  PRIVATE
  ${radar2023_source}
)
set_target_properties(radar2023 PROPERTIES
  CUDA_SEPARABLE_COMPILATION ON)
add_dependencies(radar2023 ${catkin_EXPORTED_TARGETS})
target_include_directories(radar2023
  PUBLIC
  ${catkin_INCLUDE_DIRS}
  logs)
target_link_libraries(radar2023
  ${catkin_LIBRARIES}
  ${LIBS} ${CUDA_LIBRARIES}
  ${CUDA_cudadevrt_LIBRARY}
  ${OpenCV_LIBS}
  ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_LIBRARIES}
  libmkl_rt.so 
  pthread
  libm.so
  libdl.so
  MVSDK
  Eigen3::Eigen
  cuda
  spdlog
  fmt::fmt-header-only
  cudart
  nvonnxparser
  libnvinfer.so
  libnvinfer_plugin.so
)

install(TARGETS radar2023 
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	)

install(DIRECTORY
  Config
  launch 
  logs 
  models 
  params 
  Record 
  ExpResultDir 
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
	)

macro(remove_warnings)
  add_definitions(-Wno-deprecated-declarations
    -Wno-unused-but-set-variable
    -Wno-unused-variable)
endmacro()

remove_warnings()