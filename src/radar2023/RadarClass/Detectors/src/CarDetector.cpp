#include "../include/CarDetector.h"

CarDetector::CarDetector()
{
    this->carTensorRT = new MyTensorRT_v5();
}

CarDetector::~CarDetector()
{
}

bool CarDetector::initModel()
{
    this->logger->info("CarDetector init Moudel");
    bool check = this->carTensorRT->initMyTensorRT_v5(OnnxMoudlePath_c, TensorRTEnginePath_c, Yolov5wtsPath_c, Is_p6_c, G_D_c, G_W_c, TensorRTMaxBatchSize_c, TRT_INPUT_H_c, TRT_INPUT_W_c, TRT_CLS_NUM_c, USE_FP16, TRT_kOPT_c, TRT_kMAX_c);
    if (check)
        this->logger->info("CarDetector Moudel inited");
    else
        this->logger->warn("Block CarDetector Moudel");
    return check;
}

vector<Rect> CarDetector::infer(Mat &image)
{
    vector<vector<Yolo::Detection>> results;
    vector<Mat> srcs = {image};
    results = this->carTensorRT->doInference(&srcs, 1);
    vector<Rect> final_results;
    if (results.size() == 0)
        return final_results;
    for (size_t i = 0; i < srcs.size(); ++i)
    {
        auto &res = results[i];
        for (size_t j = 0; j < res.size(); j++)
        {
            cv::Rect r = get_rect(image, res[j].bbox, TRT_INPUT_H_c, TRT_INPUT_W_c);
#ifdef Test
            cout << "Car:" << r.x << "|" << r.y << "|" << r.width << "|" << r.height <<"cls:"<< res[j].class_id << "|conf:" << res[j].conf << endl;
#endif
            final_results.emplace_back(r);
        }
    }
    return final_results;
}