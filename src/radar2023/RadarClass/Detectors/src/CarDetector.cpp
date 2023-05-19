#include "../include/CarDetector.h"

CarDetector::CarDetector()
{
}

CarDetector::~CarDetector()
{
}

bool CarDetector::initModel()
{
    this->logger->info("CarDetector init Moudel");
    if (access(TensorRTEnginePath_c, F_OK) != 0)
    {
        auto engine = this->carTensorRT.createEngine(OnnxMoudlePath_c, 4, 1280, 1280);
        this->carTensorRT.saveEngineFile(engine, TensorRTEnginePath_c);
    }
    bool check = this->carTensorRT.initModule(TensorRTEnginePath_c, 1, 1);
    if (check)
        this->logger->info("CarDetector Moudel inited");
    return check;
}

vector<Rect> CarDetector::infer(Mat &image)
{
    vector<vector<TRTInferV1::DetectionObj>> results;
    vector<Mat> srcs;
    srcs.emplace_back(image);
    results = this->carTensorRT.doInference(srcs, 0.1, 0.45, 0.3);
    vector<Rect> final_results;
    if (results.size() == 0)
        return final_results;
    for (size_t j = 0; j < results[0].size(); j++)
    {
        this->logger->info("Car: [x1] " + to_string(results[0][j].x1) + " [y1] " + to_string(results[0][j].y1) + " [x2] " + to_string(results[0][j].x2) + " [y2] " + to_string(results[0][j].y2) + " [cls] " + to_string(results[0][j].classId) + " [conf] " + to_string(results[0][j].confidence));
        final_results.emplace_back(Rect(results[0][j].x1, results[0][j].y1, results[0][j].x2 - results[0][j].x1, results[0][j].y2 - results[0][j].y1));
    }
    return final_results;
}

void CarDetector::unInit()
{
    this->carTensorRT.unInitModule();
}