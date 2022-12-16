#ifndef __MAPMAPPING_H
#define __MAPMAPPING_H

#include "../../Common/include/public.h"
#include "../../Detectors/include/depthProcesser.h"

class MapMapping
{
private:
    vector<MapLocation3D> _location3D;
    vector<MapLocation3D> cached_location3D;
    map<int, int> _ids;
    Matrix<float, 4, 4> _T;
    Matrix<float, 3, 1> cameraPostion;
    Mat rvec, tvec;
    bool _pass_flag = false;

    int _location_pred_time[10] = {0};
    vector<vector<MapLocation3D>> _location_cache = vector<vector<MapLocation3D>>(2, vector<MapLocation3D>(10, MapLocation3D()));

private:
    void adjust_z_one(MapLocation3D &locs);
    void _location_prediction();
    vector<ArmorBoundingBox> _IoU_prediction();

public:
    MapMapping();
    ~MapMapping();

    bool _is_pass();
    void push_T(Mat &rvec, Mat &tvec);
    vector<MapLocation3D> getloc();
    void mergeUpdata(vector<ArmorBoundingBox> &tensorRTbbox, vector<ArmorBoundingBox> &Ioubbox);
};

#endif