#ifndef __CARDETECTOR_H
#define __CARDETECTOR_H

#include "../../TRTInference/TRTInfer/include/Inference.h"
#include "../../Common/include/general.h"

/**
 * @brief 车辆识别
 * 车辆识别器，输出车辆BBox
 */
class CarDetector
{
private:
    TRTInferV1::TRTInfer carTensorRT = TRTInferV1::TRTInfer(0);
    std::shared_ptr<spdlog::logger> logger = spdlog::get("RadarLogger");

public:
    CarDetector();
    ~CarDetector();

    bool initModel();
    vector<Rect> infer(Mat &image);
};

#endif