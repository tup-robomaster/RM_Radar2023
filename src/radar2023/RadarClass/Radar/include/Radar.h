#ifndef __RADAR_H
#define __RADAR_H

#include "../../Common/include/public.h"
#include "../../Camera/include/camera.h"
#include "../../Camera/include/VideoRecorder.h"
#include "../../Common/include/SharedQueue.h"
#include "../../Detectors/include/depthProcesser.h"
#include "../../Detectors/include/ArmorDetector.h"
#include "../../Detectors/include/CarDetector.h"
#include "../../Location/include/MapMapping.h"
#include "../../UART/include/UART.h"
#include "../../Location/include/location.h"

/**
 * @brief 主要雷达类
 * 负责相关工作线程的管理，获取雷达点云数据并进行处理
 */
class Radar
{
private:
    bool _is_LidarInited = false;
    ros::Subscriber sub;
    thread lidarMainloop;
    thread serRead;
    thread serWrite;
    thread processLoop;
    thread videoRecoderLoop;
    bool _init_flag = false;
    bool _thread_working = false;
    bool _Ser_working = false;
    bool _CameraThread_working = false;
    bool __LidarMainLoop_working = false;
    bool __MainProcessLoop_working = false;
    bool __VideoRecorderLoop_working = false;

    DepthQueue depthQueue;
    ArmorDetector armorDetector;
    CarDetector carDetector;
    CameraThread cameraThread;
    Location myLocation;
    MapMapping mapMapping;
    UART myUART;
    MySerial mySerial;
    VideoRecorder videoRecorder;

    bool _if_record = false;

    bool is_alive = true;

    vector<vector<float>> publicDepth;
    shared_timed_mutex myMutex_publicDepth;
    shared_timed_mutex myMutex_cameraThread;
    SharedQueue<Mat> myFrames;

    Mat K_0_Mat;
    Mat C_0_Mat;
    Mat E_0_Mat;

    std::shared_ptr<spdlog::logger> logger = spdlog::get("RadarLogger");

private:
    void armor_filter(vector<bboxAndRect> &pred);
    void detectDepth(vector<bboxAndRect> &pred);
    void detectDepth(vector<ArmorBoundingBox> &armors);
    void send_judge(judge_message &message, UART &myUART);

    void drawBbox(vector<Rect> &bboxs, Mat &img);
    void drawArmorsForDebug(vector<ArmorBoundingBox> &armors, Mat &img);
    void drawArmorsForDebug(vector<bboxAndRect> &armors, Mat &img);

public:
    Radar();
    ~Radar();

    void init(int argc, char **argv);

    void LidarListenerBegin(int argc, char **argv);
    void LidarMainLoop();
    void LidarCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg);
    void SerReadLoop();
    void SerWriteLoop();
    void MainProcessLoop();
    void VideoRecorderLoop();

    void spin(int argc, char **argv);
    void stop();

    bool alive();
};

#endif