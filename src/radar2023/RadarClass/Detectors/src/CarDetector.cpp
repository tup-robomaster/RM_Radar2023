#include "../include/CarDetector.h"

CarDetector::CarDetector(string engine_path, string onnx_path)
{
    assert(!engine_path.empty() && !onnx_path.empty());
    this->TensorRTEnginePath = engine_path;
    this->OnnxPath = onnx_path;
}

CarDetector::~CarDetector()
{
}

void CarDetector::accessModelTest()
{
    if (access(TensorRTEnginePath.c_str(), F_OK) != 0)
    {
        auto engine = this->carTensorRT.createEngine(OnnxPath, 4, 1280, 1280, 0);
        this->carTensorRT.saveEngineFile(engine, TensorRTEnginePath);
        delete engine;
    }
}

bool CarDetector::initModel()
{
    this->logger->info("CarDetector init Moudel");
    bool check = this->carTensorRT.initModule(TensorRTEnginePath, 1, 1);
    if (check)
        this->logger->info("CarDetector Moudel inited");
    return check;
}

vector<DetectBox> CarDetector::infer(Mat &image)
{
    vector<vector<TRTInferV1::DetectionObj>> results;
    vector<Mat> srcs;
    srcs.emplace_back(image);
    results = this->carTensorRT.doInference(srcs, 0.1, 0.45, 0.3);
    vector<DetectBox> final_results;
    if (results.size() == 0)
        return final_results;
    for (size_t j = 0; j < results[0].size(); j++)
    {
        this->logger->info("Car: [x1] " + to_string(results[0][j].x1) +
                           " [y1] " + to_string(results[0][j].y1) +
                           " [x2] " + to_string(results[0][j].x2) +
                           " [y2] " + to_string(results[0][j].y2) +
                           " [cls] " + to_string(results[0][j].classId) +
                           " [conf] " + to_string(results[0][j].confidence));
        final_results.emplace_back(results[0][j].x1, results[0][j].y1, results[0][j].x2, results[0][j].y2, results[0][j].confidence, results[0][j].classId);
    }
    return final_results;
}

void CarDetector::unInit()
{
    this->carTensorRT.unInitModule();
}