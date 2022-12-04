#ifndef __LOCATION_H
#define __LOCATION_H

#include "public.h"
#include "camera.h"
#include "general.h"

class Location
{
private:
    map<string, Point3f> location_targets = {{}};

public:
    Location();
    ~Location();

    bool locate_pick(CameraThread &cap, int enemy, Mat &rvec_Mat, Mat &tvec_Mat);
};

#endif