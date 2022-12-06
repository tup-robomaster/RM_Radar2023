#include "../include/ArmorDetector.h"

ArmorDetector::ArmorDetector()
{
    this->armorTensorRT = new MyTensorRT();
}

ArmorDetector::~ArmorDetector()
{
}

bool ArmorDetector::initModel()
{
    fmt::print(fg(fmt::color::aqua) | fmt::emphasis::bold,
               "[INFO], {}!\n", "ArmorDetector init Moudel");
    bool check = this->armorTensorRT->initMyTensorRT(TensorRTEnginePath, Yolov5wtsPath, Is_p6, G_D, G_W, TensorRTMaxBatchSize, TRT_INPUT_H, TRT_INPUT_W, TRT_CLS_NUM);
    fmt::print(fg(fmt::color::green) | fmt::emphasis::bold,
               "[INFO], {}!\n", "ArmorDetector Moudel inited");
    return check;
}

vector<ArmorBoundingBox> ArmorDetector::infer(Mat &image, vector<Rect> targets)
{
    // TODO: Fix here
    vector<vector<Yolo::Detection>> results;
    if (targets.size() == 0)
        return {};
    vector<Mat> preProcessedImage = this->preProcess(image, targets);
    results = this->armorTensorRT->doInference(&preProcessedImage, preProcessedImage.size());
    return this->results;
}

vector<Mat> ArmorDetector::preProcess(Mat &image, vector<Rect> &movingTargets)
{
    vector<Mat> output;
    for (const auto &it : movingTargets)
    {
        output.emplace_back(image(it));
    }
    return output;
}