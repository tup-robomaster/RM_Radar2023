#ifndef __CARDETECTOR_H
#define __CARDETECTOR_H

#include "./tensorRT_v5.h"
#include "../../Common/include/general.h"

class CarDetector
{
private:
    MyTensorRT_v5 *carTensorRT;
    vector<ArmorBoundingBox> results;

public:
    CarDetector();
    ~CarDetector();

    bool initModel();
    vector<Rect> infer(Mat &image);
};

#endif