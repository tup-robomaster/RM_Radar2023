# Copyright(C),2022-2023,沈阳航空航天大学T-UP战队 All Rights Reserved

# Camera: 相机驱动模块 | 视频录制模块

## 文件结构

├── include
│   ├── camera.h   			//相机驱动头文件
│   └── VideoRecorder.h		//录制模块头文件
└── src
     ├── camera.cpp			//相机驱动cpp文件
     └── VideoRecorder.cpp	//录制模块cpp文件

## 相机驱动模块

### 1.简介

 相机驱动模块负责获取工业相机图像，并处理成OpenCV可接受的Mat格式

### 2.函数功能

CameraThread

* openCamera() 打开并初始化相机
* adjustExposure() 曝光和增益调节
* open() 尝试初始化相机驱动模块
* is_open() 检查相机驱动模块是否初始化
* read() 读取图像
* release() 释放并反初始化相机
* start() 循环尝试初始化相机驱动模块
* stop() 关闭相机驱动模块

## 视频录制模块

### 1.简介

视频录制模块负责异步将队列中的图像写入视频

### 2.函数功能

VideoRecorder

* init() 初始化视频录制模块
* write() 写入图像
* close() 关闭视频录制模块并保存视频
