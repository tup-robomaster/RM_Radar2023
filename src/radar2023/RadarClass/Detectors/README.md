# Copyright(C),2022-2023,沈阳航空航天大学T-UP战队 All Rights Reserved

# Detectors: 装甲板检测模块 | 车辆检测模块 | 点云深度图处理模块 | 点云深度图背景分割模块

## 文件结构

├── CUDA
│   ├── gpu_decode.cu       	//gpu后处理cu文件
│   ├── preprocess.cu       	//gpu预处理cu文件
│   └── yololayer.cu        		//yolo相关CUDA核函数
├── include
│   ├── ArmorDetector.h		//装甲板检测器头文件
│   ├── calibrator.h			//TRT相关头文件
│   ├── CarDetector.h		//车辆检测器头文件
│   ├── common.h			//TRT相关头文件
│   ├── cuda_utils.h			//TRT相关头文件
│   ├── depthProcesser.h		//点云深度图映射头文件
│   ├── macros.h			//TRT相关头文件
│   ├── MovementDetector.h	//点云背景分割头文件
│   ├── preprocess.h		//TRT相关头文件
│   ├── tensorRT_v5.h		//TRT主要头文件
│   ├── utils.h				//TRT相关头文件
│   └── yololayer.h			//TRT相关头文件
├── Moudles
└── src
     ├── ArmorDetector.cpp	//装甲板检测器cpp文件
     ├── calibrator.cpp		//TRT相关cpp文件
     ├── CarDetector.cpp		//车辆检测器cpp文件
     ├── common.cpp		//TRT相关cpp文件
     ├── depthProcesser.cpp	//点云深度图映射cpp文件
     ├── MovementDetector.cpp	//点云背景分割cpp文件
     └── tensorRT_v5.cpp		//TRT主要cpp文件

## 装甲板检测模块

### 1.简介

输入分割后图像，检测图像中存在的装甲板

### 2.函数功能

ArmorDetector

* initModel() 初始化Tensorrt模型
* infer() 检测并返回装甲板检测框
* preProcess() 图像分割预处理
* reBuildBoxs() 根据父检测框映射装甲板检测框至原图

## 车辆检测模块

### 1.简介

检测图像中存在的车辆，用于图像分割

### 2.函数功能

CarDetector

* initModel() 初始化Tensorrt模型
* infer() 检测并返回车辆检测框

## 点云深度图处理模块

### 1.简介

处理获取到的点云数据，积分深度图

### 2.函数功能

DepthQueue

* pushback() 输入点云，积分深度图

## 点云深度图背景分割模块

### 1.简介

处理获取到的点云数据，积分深度图

### 2.函数功能

MovementDetector

* applyMovementDetector() 应用背景分割，前期积分，后期分割
* buildBackground() 建立背景
* detectMovementTarget() 执行分割，返回Rect
* rebuildRect() 重建Rect，反栅格化
* _ifHistoryBuild() 检查背景是否建立

## 技术细节

### 1.点云深度图分割+单层神经网络预测

此方案下，由于分流了GPU运算压力，将图像分割转为CPU负载，充分利用了运算平台的富余算力，带来了进一步的性能提升。缺点是点云积分慢，分割框更新较慢且一般大于车辆实际大小，由于精度较低，所以不能使用IoU预测的方式对装甲板检测框进行消抖。

## 2.双层神经网络预测

此方案下，检测负载大部分在GPU端，且显存占用较大，总线IO负载高，但是有着最佳精度且分割框更新迅速，较为准确的车辆分割框也允许了使用IoU预测的方式进行装甲板检测框消抖，是算力足够的情况下的优选。
