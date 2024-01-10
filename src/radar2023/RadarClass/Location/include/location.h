#ifndef __LOCATION_H
#define __LOCATION_H

#include "../../Common/include/public.h"
#include "../../Camera/include/camera.h"
#include "../../Common/include/algorithm.h"

/**
 * @brief 四点标定类
 * 用于进行四点标定，求解旋转平移向量
 */
class Location
{
public:
    typedef std::shared_ptr<Location> Ptr;

public:
    vector<Point2f> pick_points;
    FrameBag frame;
    bool flag = false;
    std::shared_ptr<spdlog::logger> logger = spdlog::get("RadarLogger");
    map<string, Point3f> location_targets;
    std::vector<string> targets_selected_enemy_red;
    std::vector<string> targets_selected_enemy_blue;

public:
    Location();
    ~Location();

    bool locate_pick(CameraThread::Ptr cap, int enemy, Mat &rvec_Mat, Mat &tvec_Mat,
                     Mat &K_0, Mat &C_0, Mat &E_0);
    bool decodeMapPoints(string path);
};

#endif