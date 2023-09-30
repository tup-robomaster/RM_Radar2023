#include "../include/DsTracker.h"

static TRTLogger gLogger;

DsTracker::DsTracker(std::string sort_onnxPath, std::string sort_enginePath)
{
    this->sort_onnxPath = sort_onnxPath;
    this->sort_enginePath = sort_enginePath;
    this->DS = std::make_shared<DeepSort>(DeepSort(sort_onnxPath, sort_enginePath, 128, 256, 0, &gLogger));
}

DsTracker::~DsTracker()
{
}

void DsTracker::sort(cv::Mat &frame, vector<DetectBox> &targets)
{
    this->DS->sort(frame, targets);
}