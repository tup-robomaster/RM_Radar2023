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
    fmt::print(fg(fmt::color::aqua) | fmt::emphasis::bold,
               "[INFO], {}!\n", "CarDetector init Moudel");
    bool check = this->carTensorRT->initMyTensorRT_v5(TensorRTEnginePath_c, Yolov5wtsPath_c, Is_p6_c, G_D_c, G_W_c, TensorRTMaxBatchSize_c, TRT_INPUT_H_c, TRT_INPUT_W_c, TRT_CLS_NUM_c);
    if (check)
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
    for (auto &it : results[0])
    {
        final_results.emplace_back(get_rect(image, it.bbox, TRT_INPUT_H_c, TRT_INPUT_W_c));
    }
    return final_results;
}