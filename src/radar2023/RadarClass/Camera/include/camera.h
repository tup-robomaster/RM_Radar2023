#ifndef __CAMERA_H
#define __CAMERA_H

#include "../../Common/include/public.h"
#include "CameraApi.h"

#ifndef UsingVideo
/**
 * @brief 相机驱动类
 * 迈德威视驱动程序，提供格式化的OpenCV Mat图像
 */
class MV_Camera
{
public:
    typedef std::shared_ptr<MV_Camera> Ptr;

private:
    int iCameraCounts = 1;
    int iStatus = -1;
    tSdkCameraDevInfo tCameraEnumList[2];
    int hCamera = -1;
    tSdkCameraCapbility tCapability;
    tSdkFrameHead sFrameInfo;
    unsigned char *pFrameBuffer;
    unsigned char *pRawDataBuffer;

    std::shared_ptr<spdlog::logger> logger = spdlog::get("RadarLogger");
    string CameraConfigPath;

public:
    bool _openflag = false;

public:
    MV_Camera();
    MV_Camera(bool Is_init, string config_path);
    ~MV_Camera();

    FrameBag read();
    void uninit();

    void setExposureTime(int ex = 30);
    void setGain(int gain);
    void saveParam(const char tCameraConfigPath[23]);
    void disableAutoEx();
    int getExposureTime();
    int getAnalogGain();
};
#endif

/**
 * @brief 相机线程内部类
 * 单独驱动的相机线程内部类，提供接口以获取图像
 */
class CameraThread
{
public:
    typedef std::shared_ptr<CameraThread> Ptr;

private:
    bool _open = false;
    bool _alive = true;

#ifdef UsingVideo
    VideoCapture _cap;
    int frame_counter = 0;
    string TestVideoPath;
#else
    MV_Camera::Ptr _cap;
#endif

    bool _is_init = false;
    string CameraConfigPath;
    std::shared_ptr<spdlog::logger> logger = spdlog::get("RadarLogger");

public:
#ifndef UsingVideo
    void openCamera(bool is_init);
    void adjustExposure();
#endif

#ifdef UsingVideo
    CameraThread(string config_path, string video_path);
#else
    CameraThread(string config_path);
#endif

    ~CameraThread();
    void open();
    bool is_open();
    FrameBag read();
    void release();
    void start();
    void stop();
};

#endif