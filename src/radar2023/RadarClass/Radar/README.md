# Copyright(C),2022-2023,沈阳航空航天大学T-UP战队 All Rights Reserved

# Radar: 雷达主模块

## 0.文件结构

├── include
│   └── Radar.h		//头文件
└── src
     └── Radar.cpp		//cpp文件

## 1.简介

雷达的主要工作模块，包含各模块的初始化、各工作线程的管理和雷达点云数据的接收，通过检查标志位来控制功能运行，提供一个简易窗口进行程序供程序功能控制。

## 2.函数功能

Radar

* armor_filter() 基于id唯一过滤装甲板
* detectDepth() 根据检测框获取对应区域深度
* send_judge() 裁判系统通讯
* init() 雷达初始化
* LidarListenerBegin() 启动点云接收
* RosSpinLoop() ROS自旋线程，主要控制点云接收
* LidarCallBack() 点云数据接收回调，处理点云数据
* SeparationLoop() 图像分割工作线程
* SerReadLoop() 串口读线程
* SerWriteLoop() 串口写线程
* MainProcessLoop() 主要处理线程
* VideoRecorderLoop() 视频录制线程
* spin() 主控制循环
* stop() 关闭雷达程序
* alive() 检查雷达程序是否运行
