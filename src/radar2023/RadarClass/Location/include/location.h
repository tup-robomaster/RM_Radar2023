#ifndef __LOCATION_H
#define __LOCATION_H

#include "../../Common/include/public.h"
#include "../../Camera/include/camera.h"
#include "../../Common/include/general.h"

class Location
{
private:
    map<string, Point3f> location_targets = {{}};

public:
    vector<Point2f> pick_points;
    FrameBag frame;
    bool flag = false;

public:
    Location();
    ~Location();

    bool locate_pick(CameraThread &cap, int enemy, Mat &rvec_Mat, Mat &tvec_Mat);
};

#endif