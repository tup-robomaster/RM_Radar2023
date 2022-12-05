#ifndef __CAMERA_H
#define __CAMERA_H

#include "./public.h"
#include "CameraApi.h"

/**
 * @brief 相机驱动类
 * 迈德威视驱动程序，提供格式化的OpenCV Mat图像
 */
class MV_Camera
{
private:
    int iCameraCounts = 1;
    int iStatus = -1;
    tSdkCameraDevInfo tCameraEnumList[2];
    int hCamera = -1;
    tSdkCameraCapbility tCapability;
    tSdkFrameHead sFrameInfo;
    unsigned char *pFrameBuffer;
    unsigned char *pRawDataBuffer;

public:
    bool _openflag = false;

public:
    MV_Camera();
    MV_Camera(bool Is_init);
    ~MV_Camera();

    FrameBag read();
    void uninit();

    void setExposureTime(int ex = 30);
    void setGain(int gain);
    void saveParam(char tCameraConfigPath[23]);
    void disableAutoEx();
    int getExposureTime();
    int getAnalogGain();
};

/**
 * @brief 相机线程内部类
 * 单独驱动的相机线程内部类，提供接口以获取图像
 */
class CameraThread
{
private:
    bool _open = false;
#ifdef UsingVideo
    VideoCapture _cap;
#else
    MV_Camera _cap;
#endif
    bool _is_init = false;

public:
    void openCamera(bool is_init);
    void adjustExposure();
    CameraThread();
    ~CameraThread();
    void open();
    bool is_open();
    FrameBag read();
    void release();
    void start();
    void stop();
};

#endif