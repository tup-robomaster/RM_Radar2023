#include "../PublicInclude/CarDetector.h"

CarDetector::CarDetector()
{
}

CarDetector::~CarDetector()
{
}

bool CarDetector::initModel()
{
    fmt::print(fg(fmt::color::aqua) | fmt::emphasis::bold,
               "[INFO], {}!\n", "CarDetector init Moudel");
    bool check = this->carTensorRT->initMyTensorRT(TensorRTEnginePath_c, Yolov5wtsPath_c, Is_p6_c, G_D_c, G_W_c);
    fmt::print(fg(fmt::color::green) | fmt::emphasis::bold,
               "[INFO], {}!\n", "CarDetector Moudel inited");
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