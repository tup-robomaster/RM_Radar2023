cmake_minimum_required(VERSION 3.9.0)
SET(CMAKE_BUILD_TYPE "Release")
project(radar2023 LANGUAGES CXX CUDA)
find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(CUDA 11.8 REQUIRED)
find_package(spdlog REQUIRED)
message("CUDA_LIBRARIES:${CUDA_LIBRARIES}")
message("CUDA_INCLUDE_DIRS:${CUDA_INCLUDE_DIRS}")
include_directories()
add_compile_options(-std=c++17)
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-z,stack-size=16000000")
set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -O3  -Wall")
set(CUDA_GEN_CODE "-gencode=arch=compute_89,code=sm_89")
set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS}  -std=c++17 -O3 -Xcompiler -fPIC -g -w ${CUDA_GEN_CODE}")

catkin_package(CATKIN_DEPENDS
	roscpp
	sensor_msgs
	)
# cuda
include_directories(/data/cuda/cuda-11.8/cuda/include)
link_directories(/data/cuda/cuda-11.8/cuda/lib)
# cudnn
include_directories(/data/cuda/cuda-11.8/cudnn/v8.0.4/include)
link_directories(/data/cuda/cuda-11.8/cudnn/v8.0.4/lib64)

include_directories(
  # include
  ${catkin_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
)
include_directories(${EIGEN3_INCLUDE_DIRS})
link_directories(
  # /opt/intel/mkl/lib/intel64
  # /opt/intel/lib/intel64
  ${PCL_LIBRARY_DIRS})
cuda_add_executable(radar2023
  main.cpp
  RadarClass/TRTInference/TRTInfer/CUDA/preprocess.cu
  RadarClass/TRTInference/TRTInfer/src/Inference.cpp
  RadarClass/Camera/src/camera.cpp
  RadarClass/Camera/src/VideoRecorder.cpp
  RadarClass/Common/src/general.cpp
  RadarClass/Detectors/src/ArmorDetector.cpp
  RadarClass/Detectors/src/CarDetector.cpp
  RadarClass/Detectors/src/depthProcesser.cpp
  RadarClass/Location/src/location.cpp
  RadarClass/Location/src/MapMapping.cpp
  RadarClass/Radar/src/Radar.cpp
  RadarClass/UART/src/offical_judge_Handler.cpp
  RadarClass/UART/src/serial.cpp
  RadarClass/UART/src/UART.cpp
  RadarClass/UART/src/UARTPasser.cpp
  RadarClass/Logger/src/Logger.cpp
)
target_sources(radar2023
  PRIVATE
  main.cpp
  RadarClass/TRTInference/TRTInfer/CUDA/preprocess.cu
  RadarClass/TRTInference/TRTInfer/src/Inference.cpp
  RadarClass/Camera/src/camera.cpp
  RadarClass/Camera/src/VideoRecorder.cpp
  RadarClass/Common/src/general.cpp
  RadarClass/Detectors/src/ArmorDetector.cpp
  RadarClass/Detectors/src/CarDetector.cpp
  RadarClass/Detectors/src/depthProcesser.cpp
  RadarClass/Location/src/location.cpp
  RadarClass/Location/src/MapMapping.cpp
  RadarClass/Radar/src/Radar.cpp
  RadarClass/UART/src/offical_judge_Handler.cpp
  RadarClass/UART/src/serial.cpp
  RadarClass/UART/src/UART.cpp
  RadarClass/UART/src/UARTPasser.cpp
  RadarClass/Logger/src/Logger.cpp
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
  pthread
  libm.so
  libdl.so
  MVSDK
  Eigen3::Eigen
  cuda
  spdlog
  fmt::fmt
  cudart
  nvonnxparser
  nvinfer
)

install(TARGETS radar2023 
	ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
	RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
	)

install(DIRECTORY
  launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  # config
	)

macro(remove_warnings)
  add_definitions(-Wno-deprecated-declarations
    -Wno-unused-but-set-variable
    -Wno-unused-variable)
endmacro()

remove_warnings()