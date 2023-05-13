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
        auto engine = this->carTensorRT.createEngine(OnnxMoudlePath_c, 1, 1280, 1280);
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
    vector<Mat> srcs = {image};
    results = this->carTensorRT.doInference(srcs, 0.7, 0.8, 0.3);
    vector<Rect> final_results;
    if (results.size() == 0)
        return final_results;
    for (size_t i = 0; i < srcs.size(); ++i)
    {
        auto &res = results[i];
        for (size_t j = 0; j < res.size(); j++)
        {
#ifdef Test
            cout << "Car:" << res[j].x1 << "|" << res[j].y1 << "|" << res[j].x2 << "|" << res[j].y2 << "cls:" << res[j].classId << "|conf:" << res[j].confidence << endl;
#endif
            final_results.emplace_back(Rect(res[j].x1, res[j].y1, res[j].x2 - res[j].x1, res[j].y2 - res[j].y1));
        }
    }
    return final_results;
}