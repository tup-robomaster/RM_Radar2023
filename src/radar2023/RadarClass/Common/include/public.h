#ifndef __PUBLIC_R_H
#define __PUBLIC_R_H

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <memory>
#include <functional>
#include <unistd.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <queue>
#include "opencv2/core/eigen.hpp"
#include "opencv2/opencv.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl/common/impl/io.hpp>
#include <regex>
#include <time.h>
#include <thread>
#include <shared_mutex>
#include <mutex>
#include <future>
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>

#include "../../../config.h"
#include "../../Logger/include/Logger.h"

using namespace std;
using namespace Eigen;
using namespace cv;
using namespace pcl;
using namespace ros;

/**
 * @brief 装甲板BBox
 */
struct ArmorBoundingBox
{
    bool flag = false;
    float x0 = 0.f, y0 = 0.f, w = 0.f, h = 0.f, cls = 0.f, conf = 0.f, depth = 0.f;
};

/**
 * @brief 装甲板BBox和分割Rect绑定包
 */
struct bboxAndRect
{
    ArmorBoundingBox armor;
    Rect rect;
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
    int id;
    int x = 0, y = 0;
};

/**
 * @brief 3D坐标
 */
struct MapLocation3D
{
    bool flag = false;
    int id;
    int x = 0, y = 0, z = 0;
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

/**
 * @brief 四点标定定位点
 */
const map<string, Point3f> location_targets = {{"red_base", Point3f(1.760, -15. + 7.539, 0.200 + 0.920)},
                                               {"blue_outpost", Point3f(16.776, -15. + 12.565, 1.760)},
                                               {"red_outpost", Point3f(11.176, -15. + 2.435, 1.760)},
                                               {"blue_base", Point3f(26.162, -15. + 7.539, 0.200 + 0.920)},
                                               {"r_rt", Point3f(8.805, -5.728 - 0.660, 0.120 + 0.495)},
                                               {"r_lt", Point3f(8.805, -5.728, 0.120 + 0.495)},
                                               {"b_rt", Point3f(19.200, -9.272 + 0.660, 0.120 + 0.495)},
                                               {"b_lt", Point3f(19.200, -9.272, 0.120 + 0.495)}};

#endif