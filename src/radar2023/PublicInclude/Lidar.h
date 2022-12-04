#ifndef __LIDAR_H
#define __LIDAR_H

#include "./public.h"
#include "./camera.h"
#include "./depthProcesser.h"
#include "./MovementDetector.h"
#include "./ArmorDetector.h"
#include "./MapMapping.h"
#include "./UART.h"
#include "./location.h"

/**
 * @brief 主要雷达类
 * 负责相关工作线程的管理，获取雷达点云数据并进行处理
 */
class Lidar
{
private:
    ros::Subscriber sub;
    thread mainloop;
    thread MDloop;
    thread serRead;
    thread serWrite;
    thread processLoop;
    bool _init_flag = false;
    bool _thread_working = false;
    bool _Ser_working = false;
    promise<void> exitSignal1;
    promise<void> exitSignal2;
    promise<void> exitSignal3;

public:
    Lidar(int argc, char **argv);
    ~Lidar();

    void init(int argc, char **argv);

    void LidarListenerBegin(int argc, char **argv);
    static void LidarMainLoop(future<void> futureObj);
    void LidarCallBack(const sensor_msgs::PointCloud2::ConstPtr &msg);
    static void MovementDetectorLoop(future<void> futureObj);
    static void SerReadLoop();
    static void SerWriteLoop();
    static void MainProcessLoop(future<void> futureObj);

    void spin(int argc, char **argv);
    void stop();
};

#endif