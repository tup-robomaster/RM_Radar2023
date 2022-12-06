#ifndef __CARDETECTOR_H
#define __CARDETECTOR_H

#include "./tensorRT.h"
#include "../../Common/include/general.h"

class CarDetector
{
private:
    MyTensorRT *carTensorRT;
    vector<ArmorBoundingBox> results;

public:
    CarDetector();
    ~CarDetector();

    bool initModel();
    vector<Rect> infer(Mat &image);
};

#endif