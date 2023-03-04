cmake_minimum_required(VERSION 3.9.0)
SET(CMAKE_BUILD_TYPE "Release")
project(radar2023 LANGUAGES CXX CUDA)
find_package(OpenCV REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)
find_package(PCL REQUIRED)
find_package(CUDA 11.6 REQUIRED)
find_package(spdlog REQUIRED)
message("CUDA_LIBRARIES:${CUDA_LIBRARIES}")
message("CUDA_INCLUDE_DIRS:${CUDA_INCLUDE_DIRS}")
include_directories()
add_compile_options(-std=c++17)
set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -Wl,-z,stack-size=16000000")
set(CMAKE_CXX_FLAGS "${CMAKE_C_FLAGS} -O3  -Wall")
set(CUDA_GEN_CODE "-gencode=arch=compute_86,code=sm_86")
set(CUDA_NVCC_FLAGS "${CUDA_NVCC_FLAGS}  -std=c++17 -O3 -Xcompiler -fPIC -g -w ${CUDA_GEN_CODE}")

include_directories(

  # include
  ${catkin_INCLUDE_DIRS}
  /opt/intel/mkl/include/
  ${CUDA_INCLUDE_DIRS}
  /home/nine-fish/TensorRT/TensorRT-8.5.1.7/include
  /home/nine-fish/TensorRT/TensorRT-8.5.1.7/samples/common
)

set(TENSORRT_LIB_PATH "/home/nine-fish/TensorRT/TensorRT-8.5.1.7/lib")
file(GLOB LIBS "${TENSORRT_LIB_PATH}/*.so")
link_directories(/opt/intel/mkl/lib/intel64
  /opt/intel/lib/intel64
  ${PCL_LIBRARY_DIRS})
cuda_add_executable(radar2023
  main.cpp
  RadarClass/Camera/src/camera.cpp
  RadarClass/Camera/src/VideoRecorder.cpp
  RadarClass/Common/src/general.cpp
  RadarClass/Detectors/CUDA/gpu_decode.cu
  RadarClass/Detectors/CUDA/preprocess.cu
  RadarClass/Detectors/CUDA/yololayer.cu
  RadarClass/Detectors/src/ArmorDetector.cpp
  RadarClass/Detectors/src/calibrator.cpp
  RadarClass/Detectors/src/CarDetector.cpp
  RadarClass/Detectors/src/common.cpp
  RadarClass/Detectors/src/depthProcesser.cpp
  RadarClass/Detectors/src/MovementDetector.cpp
  RadarClass/Detectors/src/tensorRT_v5.cpp
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
  cuda
  spdlog::spdlog
)

macro(remove_warnings)
  add_definitions(-Wno-deprecated-declarations
    -Wno-unused-but-set-variable
    -Wno-unused-variable)
endmacro()

remove_warnings()