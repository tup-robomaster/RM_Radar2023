#ifndef __MAPMAPPING_H
#define __MAPMAPPING_H

#include "../../Common/include/general.h"
#include "../../Detectors/include/depthProcesser.h"

/**
 * @brief 映射类
 * 使用四点标定所得旋转平移向量进行坐标系转换
 */
class MapMapping
{
private:
    vector<MapLocation3D> _location3D;
    vector<MapLocation3D> cached_location3D;
    map<int, int> _ids = {{1, 6}, {2, 7}, {3, 8}, {4, 9}, {5, 10}, {8, 1}, {9, 2}, {10, 3}, {11, 4}, {12, 5}};
    Matrix<float, 4, 4> _T;
    Matrix<float, 3, 1> cameraPostion;
    Mat rvec, tvec;
    bool _pass_flag = false;

    int _location_pred_time[10] = {0};
    vector<vector<MapLocation3D>> _location_cache;
    vector<bboxAndRect> _IoU_pred_cache;
    std::shared_ptr<spdlog::logger> logger = spdlog::get("RadarLogger");

private:
    void adjust_z_one(MapLocation3D &locs);
    void _location_prediction();

public:
    MapMapping();
    ~MapMapping();

    bool _is_pass();
    void push_T(Mat &rvec, Mat &tvec);
    void _plot_region_rect(vector<Point3f> &points, Mat &frame, Mat &K_0, Mat &C_0);
    vector<ArmorBoundingBox> _IoU_prediction(vector<bboxAndRect> pred, vector<Rect> sepboxs);
    vector<MapLocation3D> getloc();
    void mergeUpdata(vector<bboxAndRect> &pred, vector<ArmorBoundingBox> &Ioubbox, int &sepMode);
};

#endif