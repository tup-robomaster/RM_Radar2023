#define EIGEN_USE_MKL_ALL
#define EIGEN_VECTORIZE_SSE4_2

#ifndef __PUBLIC_H
#define __PUBLIC_H

#include <stdio.h>
#include <iostream>
#include <fstream>
#include <string>
#include <memory>
#include <functional>
#include <unistd.h>
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
#include <fmt/format.h>
#include <fmt/color.h>

#include "../../../config.h"

using namespace std;
using namespace Eigen;
using namespace cv;
using namespace pcl;
using namespace ros;

/**
 * @brief 简化的装甲板BBox
 */
struct ArmorBoundingBox
{
    bool flag = false;
    float x0, y0, w, h, cls, conf, depth;
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
 * @brief 映射绑定包
 */
struct ReShapeBox
{
    int index;
    int x1, y1, x2, y2;
    float dw, dh;
    float r_w;
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
 * @brief 预警信息
 */
struct AlarmBag
{
    bool flag = false;
    unsigned char code, team;
    vector<unsigned char> send_targets, alarm_targets;
};

/**
 * @brief 比赛信息包
 */
struct BOData
{
    bool GameEndFlag = false;
    int remainBO = -1;
};

struct judge_message
{
    int task;
    vector<unsigned char> targets;
    unsigned char team;
    vector<MapLocation3D> loc;
};
const map<string, Point3f> location_targets = {{"red_base", Point3f(1.760, -15. + 7.539, 0.200 + 0.920)},
                                         {"blue_outpost", Point3f(16.776, -15. + 12.565, 1.760)},
                                         {"red_outpost", Point3f(11.176, -15. + 2.435, 1.760)},
                                         {"blue_base", Point3f(26.162, -15. + 7.539, 0.200 + 0.920)},
                                         {"r_rt", Point3f(8.805, -5.728 - 0.660, 0.120 + 0.495)},
                                         {"r_lt", Point3f(8.805, -5.728, 0.120 + 0.495)},
                                         {"b_rt", Point3f(19.200, -9.272 + 0.660, 0.120 + 0.495)},
                                         {"b_lt", Point3f(19.200, -9.272, 0.120 + 0.495)}};

#endif