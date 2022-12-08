#ifndef __RADAR_H
#define __RADAR_H

#include "../../Common/include/public.h"
#include "../../Camera/include/camera.h"
#include "../../Camera/include/VideoRecoder.h"
#include "../../Common/include/SharedQueue.h"
#include "../../Detectors/include/depthProcesser.h"
#include "../../Detectors/include/MovementDetector.h"
#include "../../Detectors/include/ArmorDetector.h"
#include "../../Detectors/include/CarDetector.h"
#include "../../Radar/include/MapMapping.h"
#include "../../UART/include/UART.h"
#include "../../Radar/include/location.h"

/**
 * @brief 主要雷达类
 * 负责相关工作线程的管理，获取雷达点云数据并进行处理
 */
class Radar
{
private:
    bool _is_LidarInited = false;
    ros::Subscriber sub;
    thread mainloop;
    thread Seqloop;
    thread serRead;
    thread serWrite;
    thread processLoop;
    thread videoRecoderLoop;
    bool _init_flag = false;
    bool _thread_working = false;
    bool _Ser_working = false;
    promise<void> exitSignal1;
    promise<void> exitSignal2;
    promise<void> exitSignal3;
    promise<void> exitSignal4;

    bool carInferAvailable = false;

    bool is_alive = true;

public:
    Radar(int argc, char **argv);
    ~Radar();

    void init(int argc, char **argv);

    void LidarListenerBegin(int argc, char **argv);
    static void LidarMainLoop(future<void> futureObj);
    void LidarCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg);
    static void SeparationLoop(future<void> futureObj);
    static void SerReadLoop();
    static void SerWriteLoop();
    static void MainProcessLoop(future<void> futureObj);
    static void VideoRecoderLoop(future<void> futureObj);

    void spin(int argc, char **argv);
    void stop();

    bool alive();
};

#endif