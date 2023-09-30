#ifndef __TRACKER_H
#define __TRACKER_H

#include "../../Common/include/public.h"
#include "deepsort.h"

class DsTracker
{
private:
    std::string sort_onnxPath;
    std::string sort_enginePath;
    std::shared_ptr<DeepSort> DS;
    std::shared_ptr<spdlog::logger> logger = spdlog::get("RadarLogger");

public:
    DsTracker(std::string sort_onnxPath, std::string sort_enginePath);
    ~DsTracker();

    void sort(cv::Mat &frame, vector<DetectBox> &targets);
};

#endif