#ifndef __ARMORDETECTOR_H
#define __ARMORDETECTOR_H

#include "./tensorRT.h"
#include "../../Common/include/general.h"

/**
 * @brief 装甲板识别
 * 装甲板识别器，输出装甲板BBox
 */
class ArmorDetector
{
private:
    MyTensorRT *armorTensorRT;
    vector<ArmorBoundingBox> results;

private:
    vector<Mat> preProcess(Mat &image, vector<Rect> &movingTargets);
    void reBuildBoxs(vector<vector<Yolo::Detection>> &armors, vector<Rect> &boxs, vector<Mat> &img);

public:
    ArmorDetector();
    ~ArmorDetector();

    bool initModel();
    vector<ArmorBoundingBox> infer(Mat &image, vector<Rect> &targets);
};

#endif