#ifndef DEEPSORT_H
#define DEEPSORT_H

#include "../../Common/include/public.h"
#include "featuretensor.h"
#include "tracker.h"
#include "datatype.h"

using nvinfer1::ILogger;
using std::vector;

class DeepSort
{
public:
    DeepSort(std::string onnx_path, std::string engine_path, int batchSize, int featureDim, int gpuID, ILogger *gLogger);
    ~DeepSort();

public:
    void sort(cv::Mat &frame, vector<DetectBox> &dets);

private:
    void sort(cv::Mat &frame, DETECTIONS &detections);
    void sort(cv::Mat &frame, DETECTIONSV2 &detectionsv2);
    void sort(vector<DetectBox> &dets);
    void sort(DETECTIONS &detections);
    void init();

private:
    std::string onnx_path;
    std::string engine_path;
    int batchSize;
    int featureDim;
    cv::Size imgShape;
    float confThres;
    float nmsThres;
    int maxBudget;
    float maxCosineDist;

private:
    vector<RESULT_DATA> result;
    vector<std::pair<CLSCONF, DETECTBOX>> results;
    tracker *objTracker;
    FeatureTensor *featureExtractor;
    ILogger *gLogger;
    int gpuID;
};

#endif // deepsort.h
