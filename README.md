# RM_Radar2023

沈阳航空航天大学T-UP战队2023赛季雷达程序

## Version: V0.3 Alpha

程序点云接收基于ros-noetic框架

## 0.前言

本赛季出于对后续扩展性考虑，原有雷达程序性能已无法满足继续开发的需求，故进行重构。程序整体的设计目的在于尽可能的压榨算力的条件下保留充足的可扩展性，各模块充分解耦、即插即用，做到程序二次开发极其简便、扩展性高。

## 1.简介

本程序适用采用激光雷达+单工业相机方案的雷达岗。硬件上我们采用Livox 觅道Mid-70激光雷达和迈德威视MV-SUA630C-T工业相机，串口通讯采用了USB转TTL的方式。根据实际场景，可以选择快速但低精度的简易点云背景分割+单层神经网络方案（CPU+GPU均衡负载）或速度略低但精度较高的双层神经网络方案（GPU重负载CPU轻负载）。

功能：提供场上车辆的高精度坐标

部署前需要自行测量的量并更改的量有：

* 相机内参与畸变参数
* 相机到激光雷达的外参
* 网络对应的TensorRT相关参数

对每个模块更加具体的说明可以参考文件夹下的README.md文件:

> [Camera](src/radar2023/RadarClass/Camera/README.md)
> [Detectors](src/radar2023/RadarClass/Detectors/README.md)
> [Location](src/radar2023/RadarClass/Location/README.md)
> [Radar](src/radar2023/RadarClass/Radar/README.md)

## 2.环境配置

1[recommend]

* Ubuntu 20.04 LTS
* GCC 9.3.0
* CUDA 11.6
* cudnn 8.6.0
* Tensorrt 8.5.1.7
* ros-noetic
* OpenCV4.6.0
* PCL
* spdlog
* Eigen3
* MKL
* Livox雷达驱动
* 迈德威视相机驱动

运算平台 (recommend)：

* AMD R7 5800H CPU
* 32G RAM
* GeForce RTX 3070 Laptop GPU

2[best]

* Ubuntu 20.04 LTS
* GCC 9.4.0
* CUDA 11.8
* cudnn 8.9.0
* Tensorrt 8.5.2.2
* ros-noetic
* OpenCV4.6.0
* PCL
* spdlog
* Eigen3
* MKL
* Livox雷达驱动
* 迈德威视相机驱动

运算平台 (best)：

* i9 13900KF
* 32G RAM
* GeForce RTX 4090 GPU

## 3.文件结构

* radar2023
  * demo_resource 存放常用资源和工具
  * logs 日志存放文件夹
  * RadarClass 雷达模块库
    * Camera 相机及录制相关
    * Common 通用文件
    * Detectors 检测器及Tensorrt
    * Location 坐标系处理相关
    * Logger spdlog日志记录器
    * Radar 程序主线程及线程管理
    * UART 官方裁判系统通讯
  * Recorder 录制文件存放
  * CMakeLists.txt CMake文件
  * config.h 程序参数配置文件
  * main.cpp 程序主文件
  * package.xml ROS

## 4.其他

### 使用前准备

* 环境配置完成后，需根据运算平台及环境修改src下CMakeLists.txt
* 创建以下文件夹：src/radar2023/logs 、src/radar2023/Recorder 、  src/radar2023/RadarClass/Detectors/Moudles  、 src/radar2023/RadarClass/Camera/params
* 修改config.h中的路径
* 准备装甲板识别及车辆识别模型，现版本可用模型为yolov5 v6.0，注意导出动态Onnx
* 车辆分类[CAR]
* 装甲板分类[B1 B2 B3 B4 B5 B6 B7 R1 R2 R3 R4 R5 R7 N1 N2 N3 N4 N5 N7 P1 P2 P3 P4 P5 P7]
* 将yolov5导出的动态尺寸onnx放置于config.h定义位置
* 将标定所得参数放置在src/radar2023/RadarClass/Camera/params文件夹中，格式如camera0.yaml所示
* 确保ROS环境激活后在RM_RADAR2023文件夹下使用：

  ```
  catkin_make
  ```
* 为串口及雷达驱动添加权限

### 使用

```
source devel/setup.bash
roslaunch radar2023 radar2023.launch
```

### 开发日志

Date:2023.5.13 更换推理模块，删除废弃功能，发布V0.3a测试版本

Date:2022-12-31 经过一轮调试Debug与更新，发布V0.2a测试版本

Date:2022-12-04 完成所有基本功能构建，发布V0.1a测试版本
