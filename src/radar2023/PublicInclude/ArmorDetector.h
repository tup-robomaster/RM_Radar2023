#ifndef __ARMORDETECTOR_H
#define __ARMORDETECTOR_H

#include "tensorRT.h"
#include "general.h"

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
    Mat letterBoxCPU(cv::Mat &src, int h, int w, ReShapeBox *box);
    vector<Mat> preProcess(Mat &image, vector<Rect> &movingTargets, vector<ReShapeBox> *boxs);
    void reBuildBoxs(vector<vector<Yolo::Detection>> *armors, vector<ReShapeBox> *boxs);

public:
    ArmorDetector();
    ~ArmorDetector();

    void initModel();
    vector<ArmorBoundingBox> infer(Mat &image, vector<Rect> targets);
};

#endif