#include "../include/ArmorDetector.h"

ArmorDetector::ArmorDetector()
{
    this->armorTensorRT = new MyTensorRT_v5();
}

ArmorDetector::~ArmorDetector()
{
}

bool ArmorDetector::initModel()
{
    fmt::print(fg(fmt::color::aqua) | fmt::emphasis::bold,
               "[INFO], {}!\n", "ArmorDetector init Moudel");
    bool check = this->armorTensorRT->initMyTensorRT_v5(TensorRTEnginePath, Yolov5wtsPath, Is_p6, G_D, G_W, TensorRTMaxBatchSize, TRT_INPUT_H, TRT_INPUT_W, TRT_CLS_NUM);
    fmt::print(fg(fmt::color::green) | fmt::emphasis::bold,
               "[INFO], {}!\n", "ArmorDetector Moudel inited");
    return check;
}

vector<bboxAndRect> ArmorDetector::infer(Mat &image, vector<Rect> &targets)
{
    vector<vector<Yolo::Detection>> results_pre;
    if (targets.size() == 0)
        return {};
    vector<Mat> preProcessedImage = this->preProcess(image, targets);
    results_pre = this->armorTensorRT->doInference(&preProcessedImage, preProcessedImage.size());
    this->reBuildBoxs(results_pre, targets, preProcessedImage);
    return this->results;
}

vector<Mat> ArmorDetector::preProcess(Mat &image, vector<Rect> &movingTargets)
{
    vector<Mat> output;
    for (const auto &it : movingTargets)
    {
        output.emplace_back(image(it));
    }
    return output;
}

void ArmorDetector::reBuildBoxs(vector<vector<Yolo::Detection>> &armors, vector<Rect> &boxs, vector<Mat> &img)
{
    vector<bboxAndRect>().swap(this->results);
    if (armors.size() != boxs.size())
        return;
    for (size_t i = 0; i < boxs.size(); ++i)
    {
        for (auto &it : armors[i])
        {
            Rect result = get_rect(img[i], it.bbox, TRT_INPUT_H, TRT_INPUT_W);
            this->results.emplace_back(bboxAndRect{ArmorBoundingBox{true, (float)result.x + boxs[i].x, (float)result.y + boxs[i].y, (float)result.width, (float)result.height, it.class_id, it.conf}, boxs[i]});
        }
    }
}