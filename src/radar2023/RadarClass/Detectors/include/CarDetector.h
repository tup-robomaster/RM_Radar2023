#ifndef __CARDETECTOR_H
#define __CARDETECTOR_H

#include "../../TRTInference/TRTInfer/include/Inference.h"
#include "../../Common/include/algorithm.h"

/**
 * @brief 车辆识别
 * 车辆识别器，输出车辆BBox
 */
class CarDetector
{
public:
    typedef std::shared_ptr<CarDetector> Ptr;

private:
    TRTInferV1::TRTInfer carTensorRT = TRTInferV1::TRTInfer(0);
    std::shared_ptr<spdlog::logger> logger = spdlog::get("RadarLogger");

    string TensorRTEnginePath;
    string OnnxPath;

public:
    CarDetector(string engine_path, string onnx_path);
    ~CarDetector();

    void accessModelTest();
    bool initModel();
    vector<DetectBox> infer(Mat &image);
    void unInit();
};

#endif