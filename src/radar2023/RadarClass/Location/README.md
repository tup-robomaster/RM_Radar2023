# Copyright(C),2022-2023,沈阳航空航天大学T-UP战队 All Rights Reserved

# Location: 手动四点标定模块 | 坐标映射模块

## 文件结构

├── include
│   ├── location.h			//手动四点标定头文件
│   └── MapMapping.h		//坐标映射头文件
└── src
     ├── location.cpp			//手动四点标定cpp文件
     └── MapMapping.cpp		//坐标映射cpp文件

## 手动四点标定模块

### 1.简介

进行四点标定，计算相机坐标系到世界坐标系的旋转平移矩阵

### 2.函数功能

Location

* locate_pick() 进行标定，返回是否成功

## 坐标映射模块

### 1.简介

根据旋转平移矩阵，进行相机坐标系到世界坐标系的坐标变换，同时进行消抖

### 2.函数功能

MapMapping

* _is_pass() 检查是否可以进行映射
* _location_prediction() 2D坐标预测，用于消抖
* _plot_region_rect() 坐标系变换并绘制区域
* _IoU_prediction() 检测框预测，用于消抖
* push_T() 输入旋转平移矩阵
* getloc() 获取坐标
* mergeUpdata() 传感器信息融合更新
