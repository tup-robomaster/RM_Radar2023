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
    bool check = this->carTensorRT.initMoudle(TensorRTEnginePath_c, 1, 1);
    if (check)
        this->logger->info("CarDetector Moudel inited");
    return check;
}

vector<Rect> CarDetector::infer(Mat &image)
{
    vector<vector<TRTInferV1::DetectionObj>> results;
    vector<Mat> srcs;
    srcs.emplace_back(image);
    results = this->carTensorRT.doInference(srcs, 0.6, 0.1, 0.3);
    vector<Rect> final_results;
    if (results.size() == 0)
        return final_results;
    for (size_t j = 0; j < results[0].size(); j++)
    {
#ifdef Test
        cout << "Car:" << results[0][j].x1 << "|" << results[0][j].y1 << "|" << results[0][j].x2 << "|" << results[0][j].y2 << "cls:" << results[0][j].classId << "|conf:" << results[0][j].confidence << endl;
#endif
        final_results.emplace_back(Rect(results[0][j].x1, results[0][j].y1, results[0][j].x2 - results[0][j].x1, results[0][j].y2 - results[0][j].y1));
    }
    return final_results;
}