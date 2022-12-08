#include "../include/CarDetector.h"

CarDetector::CarDetector()
{
    this->carTensorRT = new MyTensorRT();
}

CarDetector::~CarDetector()
{
}

bool CarDetector::initModel()
{
    fmt::print(fg(fmt::color::aqua) | fmt::emphasis::bold,
               "[INFO], {}!\n", "CarDetector init Moudel");
    bool check = this->carTensorRT->initMyTensorRT(TensorRTEnginePath_c, Yolov5wtsPath_c, Is_p6_c, G_D_c, G_W_c, TensorRTMaxBatchSize_c, TRT_INPUT_H_c, TRT_INPUT_W_c, TRT_CLS_NUM_c);
    if(check)
        fmt::print(fg(fmt::color::green) | fmt::emphasis::bold,
                "[INFO], {}!\n", "CarDetector Moudel inited");
    else
        fmt::print(fg(fmt::color::yellow) | fmt::emphasis::bold,
                "[WARN], {}!\n", "Block CarDetector Moudel");
    return check;
}

vector<Rect> CarDetector::infer(Mat &image)
{
    vector<vector<Yolo::Detection>> results;
    vector<Mat> srcs = {image};
    results = this->carTensorRT->doInference(&srcs, 1);
    vector<Rect> final_results;
    for (const auto &it : results[0])
    {
        final_results.emplace_back(Rect(it.bbox[0] - it.bbox[2] / 2, it.bbox[1] - it.bbox[3] / 2, it.bbox[2], it.bbox[3]));
    }
    return final_results;
}