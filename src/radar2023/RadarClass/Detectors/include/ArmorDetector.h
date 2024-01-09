#ifndef __ARMORDETECTOR_H
#define __ARMORDETECTOR_H

#include "../../TRTInference/TRTInfer/include/Inference.h"
#include "../../Common/include/algorithm.h"

/**
 * @brief 装甲板识别
 * 装甲板识别器，输出装甲板BBox和对应分割框
 */
class ArmorDetector
{
public:
    typedef std::shared_ptr<ArmorDetector> Ptr;

private:
    TRTInferV1::TRTInfer armorTensorRT = TRTInferV1::TRTInfer(0);
    vector<bboxAndRect> results;
    std::shared_ptr<spdlog::logger> logger = spdlog::get("RadarLogger");

    string TensorRTEnginePath;
    string OnnxPath;

#ifdef ExperimentalOutput
    int detectedArmorNumThisFrame = 0;
    float averageConfThisFrame = 0.0;
#endif

private:
    vector<Mat> preProcess(Mat &image, vector<DetectBox> &movingTargets);
    void reBuildBoxs(vector<vector<TRTInferV1::DetectionObj>> &armors, vector<DetectBox> &boxs, vector<Mat> &img);

public:
    ArmorDetector(string engine_path, string onnx_path);
    ~ArmorDetector();

    void accessModelTest();
    bool initModel();
    vector<bboxAndRect> infer(Mat &image, vector<DetectBox> &targets);
    void unInit();

#ifdef UseOneLayerInfer
    vector<bboxAndRect> infer(Mat &image);
#endif
};

#endif