#ifndef __CARDETECTOR_H
#define __CARDETECTOR_H

#include "./tensorRT_v5.h"
#include "../../Common/include/general.h"

/**
 * @brief 车辆识别
 * 车辆识别器，输出车辆BBox
 */
class CarDetector
{
private:
    MyTensorRT_v5 *carTensorRT;
    vector<ArmorBoundingBox> results;
    std::shared_ptr<spdlog::logger> logger = spdlog::get("RadarLogger");

public:
    CarDetector();
    ~CarDetector();

    bool initModel();
    vector<Rect> infer(Mat &image);
};

#endif