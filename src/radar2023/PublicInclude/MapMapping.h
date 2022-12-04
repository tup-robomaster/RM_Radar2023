#ifndef __MAPMAPPING_H
#define __MAPMAPPING_H

#include "./public.h"
#include "./depthProcesser.h"

class MapMapping
{
private:
    vector<MapLocation3D> _location3D;
    vector<MapLocation3D> cached_location3D;
    map<int, int> _ids;
    Matrix<float, 4, 4> _T;
    Mat revc, tvec;
    bool _pass_flag = false;

public:
    MapMapping();
    ~MapMapping();

    bool _is_pass();
    void push_T(Mat revc, Mat tevc);
    vector<MapLocation3D> updata();
    void mergeUpdata(vector<ArmorBoundingBox> &tensorRTbbox, vector<ArmorBoundingBox> &Ioubbox);
    void adjust_z_one(vector<MapLocation3D> &locs);
};

#endif