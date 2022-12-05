#include "../PublicInclude/ArmorDetector.h"

ArmorDetector::ArmorDetector()
{
    this->armorTensorRT = new MyTensorRT();
}

ArmorDetector::~ArmorDetector()
{
}

void ArmorDetector::initModel()
{
    fmt::print(fg(fmt::color::aqua) | fmt::emphasis::bold,
               "[INFO], {}!\n", "ArmorDetector init Moudel");
    this->armorTensorRT->initMyTensorRT(TensorRTEnginePath, Yolov5wtsPath);
    fmt::print(fg(fmt::color::green) | fmt::emphasis::bold,
               "[INFO], {}!\n", "ArmorDetector Moudel inited");
}

vector<ArmorBoundingBox> ArmorDetector::infer(Mat &image, vector<Rect> targets)
{
    vector<vector<Yolo::Detection>> results;
    vector<ReShapeBox> boxs;
    if(targets.size() == 0)
        return {};
    vector<Mat> preProcessedImage = this->preProcess(image, targets, &boxs);
    results = this->armorTensorRT->doInference(&preProcessedImage, preProcessedImage.size());
    if (results.size() == 0)
        return {};
    this->reBuildBoxs(&results, &boxs);
    return this->results;
}

Mat ArmorDetector::letterBoxCPU(cv::Mat &src, int h, int w, ReShapeBox *box)
{
    int in_w = src.cols;
    int in_h = src.rows;
    int tar_w = w;
    int tar_h = h;
    float r = min(float(tar_h) / in_h, float(tar_w) / in_w);
    int inside_w = round(in_w * r);
    int inside_h = round(in_h * r);
    float padd_w = tar_w - inside_w;
    float padd_h = tar_h - inside_h;
    cv::Mat resize_img;
    resize(src, resize_img, cv::Size(inside_w, inside_h));
    padd_w = padd_w / 2;
    padd_h = padd_h / 2;
    box->dh = padd_h;
    box->dw = padd_w;
    box->r_w = r;
    copyMakeBorder(resize_img, resize_img, int(round(padd_h - 0.1)), int(round(padd_h + 0.1)), int(round(padd_w - 0.1)), int(round(padd_w + 0.1)), 0, cv::Scalar(114, 114, 114));
    return resize_img;
}

vector<Mat> ArmorDetector::preProcess(Mat &image, vector<Rect> &movingTargets, vector<ReShapeBox> *boxs)
{
    vector<Mat> output;
    vector<Mat> temp;
    int index = 0;
    for (const auto &it : movingTargets)
    {
        if (index == 4)
            index = 0;
        ReShapeBox box;
        box.index = index;
        box.x1 = it.x;
        box.x2 = it.x + it.width;
        box.y1 = it.y;
        box.y2 = it.y + it.height;
        Mat image_ROI(image(it));
        temp.emplace_back(letterBoxCPU(image_ROI, TRT_INPUT_H / 2, TRT_INPUT_W / 2, &box));
        boxs->emplace_back(box);
        ++index;
        if (temp.size() == 4 || &it == &movingTargets.back())
        {
            Mat outputMat(TRT_INPUT_W, TRT_INPUT_H, CV_8UC3, Scalar(114, 114, 114));
            int i = 0;
            for (const auto &jt : temp)
            {
                switch (i)
                {
                case 0:
                    jt.copyTo(outputMat(Rect(0, 0, jt.cols, jt.rows)));
                    break;
                case 1:
                    jt.copyTo(outputMat(Rect(TRT_INPUT_W / 2, 0, jt.cols, jt.rows)));
                    break;
                case 2:
                    jt.copyTo(outputMat(Rect(0, TRT_INPUT_H / 2, jt.cols, jt.rows)));
                    break;
                case 3:
                    jt.copyTo(outputMat(Rect(TRT_INPUT_W / 2, TRT_INPUT_H / 2, jt.cols, jt.rows)));
                    break;
                default:
                    break;
                }
                ++i;
            }
            output.emplace_back(outputMat);
            temp.clear();
        }
    }
    return output;
}

void ArmorDetector::reBuildBoxs(vector<vector<Yolo::Detection>> *armors, vector<ReShapeBox> *boxs)
{
    vector<ArmorBoundingBox>().swap(this->results);
    for (size_t i = 0; i < armors->size(); i++)
    {
        for (const auto &it : armors->at(i))
        {
            float x1, x2, y1, y2;
            ArmorBoundingBox tempBox;
            ReShapeBox box;
            bool flag = false;
            if (it.bbox[0] < TRT_INPUT_W / 2 && it.bbox[1] < TRT_INPUT_H / 2)
            {
                box = boxs->at(floor(i / 4));
                x1 = it.bbox[0] - it.bbox[2] / 2;
                y1 = it.bbox[1] - it.bbox[3] / 2;
                x2 = it.bbox[0] + it.bbox[2] / 2;
                y2 = it.bbox[1] + it.bbox[3] / 2;
                flag = true;
            }
            if (it.bbox[0] > TRT_INPUT_W / 2 && it.bbox[1] < TRT_INPUT_H / 2)
            {
                box = boxs->at(floor(i / 4) + 1);
                x1 = it.bbox[0] - it.bbox[2] / 2 - TRT_INPUT_W / 2;
                y1 = it.bbox[1] - it.bbox[3] / 2;
                x2 = it.bbox[0] + it.bbox[2] / 2 - TRT_INPUT_W / 2;
                y2 = it.bbox[1] + it.bbox[3] / 2;
                flag = true;
            }
            if (it.bbox[0] < TRT_INPUT_W / 2 && it.bbox[1] > TRT_INPUT_H / 2)
            {
                box = boxs->at(floor(i / 4) + 2);
                x1 = it.bbox[0] - it.bbox[2] / 2;
                y1 = it.bbox[1] - it.bbox[3] / 2 - TRT_INPUT_H / 2;
                x2 = it.bbox[0] + it.bbox[2] / 2;
                y2 = it.bbox[1] + it.bbox[3] / 2 - TRT_INPUT_H / 2;
                flag = true;
            }
            if (it.bbox[0] > TRT_INPUT_W / 2 && it.bbox[1] > TRT_INPUT_H / 2)
            {
                box = boxs->at(floor(i / 4) + 3);
                x1 = it.bbox[0] - it.bbox[2] / 2 - TRT_INPUT_W / 2;
                y1 = it.bbox[1] - it.bbox[3] / 2 - TRT_INPUT_H / 2;
                x2 = it.bbox[0] + it.bbox[2] / 2 - TRT_INPUT_W / 2;
                y2 = it.bbox[1] + it.bbox[3] / 2 - TRT_INPUT_H / 2;
                flag = true;
            }
            if (flag)
            {
                tempBox.x0 = box.x1 + (x1 - box.dw / 2) / box.r_w;
                tempBox.y0 = box.y1 + (y1 - box.dh / 2) / box.r_w;
                tempBox.w = (x2 - x1) / box.r_w;
                tempBox.h = (y2 - y1) / box.r_w;
                tempBox.cls = it.class_id;
                tempBox.conf = it.conf;
                this->results.emplace_back(tempBox);
            }
        }
    }
}