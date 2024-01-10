#ifndef __PUBLIC_R_H
#define __PUBLIC_R_H
#define EIGEN_USE_MKL_ALL
#define EIGEN_VECTORIZE_SSE4_2

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <memory>
#include <functional>
#include <unistd.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <queue>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/common/impl/io.hpp>
#include <regex>
#include <time.h>
#include <thread>
#include <shared_mutex>
#include <mutex>
#include <future>
#include <ros/ros.h>
#include <ros/package.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "../../../config.h"
#include "../../Logger/include/Logger.h"

#include "../../../../ThirdParty/Json/json.h"

using namespace std;
using namespace Eigen;
using namespace cv;
using namespace pcl;
using namespace ros;

#ifdef __cplusplus
    extern "C++"
    {
        template <typename _Ty, size_t _Size>
        char (*__countof_helper(_Ty (&arr)[_Size]))[_Size];

        #define __crt_countof(arr) (sizeof(*__countof_helper(arr)) + 0)
    }
#endif

/**
 * @brief 一层网络检测框
 */
typedef struct DetectBox {
    DetectBox(float x1=0, float y1=0, float x2=0, float y2=0, 
            float confidence=0, float classID=-1, float trackID=-1) {
        this->x1 = x1;
        this->y1 = y1;
        this->x2 = x2;
        this->y2 = y2;
        this->confidence = confidence;
        this->classID = classID;
        this->trackID = trackID;
    }
    float x1, y1, x2, y2;
    float confidence;
    float classID;
    float trackID;
} DetectBox;

/**
 * @brief 装甲板BBox
 */
struct ArmorBoundingBox
{
    bool flag = false;
    _Float32 x0 = 0.f, y0 = 0.f, w = 0.f, h = 0.f, cls = 0.f, conf = 0.f, depth = 0.f;
};

/**
 * @brief 装甲板BBox和分割Rect绑定包
 */
struct bboxAndRect
{
    ArmorBoundingBox armor;
    DetectBox rect;
};

/**
 * @brief 相机数据绑定包
 */
struct FrameBag
{
    bool flag = false;
    Mat frame;
};

/**
 * @brief 2D坐标
 */
struct MapLocation2D
{
    bool flag = false;
    float depth;
    int id = -1;
    int x = 0, y = 0;
};

/**
 * @brief 3D坐标
 */
struct MapLocation3D
{
    bool flag = false;
    int id = -1;
    float x = 0., y = 0., z = 0.;
};

/**
 * @brief 比赛信息包
 */
struct BOData
{
    bool GameEndFlag = false;
    int remainBO = -1;
};

/**
 * @brief 裁判系统信息包
 */
struct judge_message
{
    int task;
    vector<unsigned char> targets;
    unsigned char team;
    vector<MapLocation3D> loc;
};

#endif