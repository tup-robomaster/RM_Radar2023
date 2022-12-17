#ifndef __RADAR_H
#define __RADAR_H

#include "../../Common/include/public.h"
#include "../../Camera/include/camera.h"
#include "../../Camera/include/VideoRecorder.h"
#include "../../Common/include/SharedQueue.h"
#include "../../Detectors/include/depthProcesser.h"
#include "../../Detectors/include/MovementDetector.h"
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
    thread seqloop;
    thread serRead;
    thread serWrite;
    pthread_t serR_t;
    pthread_t serW_t;
    thread processLoop;
    thread videoRecoderLoop;
    bool _init_flag = false;
    bool _thread_working = false;
    bool _Ser_working = false;

    bool __LidarMainLoop_working = false;
    bool __SeparationLoop_working = false;
    bool __MainProcessLoop_working = false;
    bool __VideoRecorderLoop_working = false;

    DepthQueue depthQueue;
    MovementDetector movementDetector;
    ArmorDetector armorDetector;
    CarDetector carDetector;
    CameraThread cameraThread;
    MapMapping mapMapping;
    UART myUART;
    MySerial mySerial;
    VideoRecorder videoRecorder;

    bool carInferAvailable = false;

    bool is_alive = true;

    vector<vector<float>> publicDepth; // 共享深度图
    int depthResourceCount;            // 深度图资源计数
    shared_timed_mutex myMutex;        // 读写锁
    vector<Rect> SeqTargets;           // 共享分割目标
    int separation_mode = 0;           // 图像分割模式
    SharedQueue<Mat> myFrames;         // 图像帧队列

private:
    void armor_filter(vector<ArmorBoundingBox> &armors);
    void detectDepth(vector<ArmorBoundingBox> &armorBoundingBoxs);
    void send_judge(judge_message &message, UART &myUART);

public:
    Radar(int argc, char **argv);
    ~Radar();

    void init(int argc, char **argv);

    void LidarListenerBegin(int argc, char **argv);
    static void LidarMainLoop(Radar *radar);
    void LidarCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg);
    static void SeparationLoop(Radar *radar);
    static void SerReadLoop(Radar *radar);
    static void SerWriteLoop(Radar *radar);
    static void MainProcessLoop(Radar *radar);
    static void VideoRecorderLoop(Radar *radar);

    void spin(int argc, char **argv);
    void stop();

    bool alive();
};

#endif