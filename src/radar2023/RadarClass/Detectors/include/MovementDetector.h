#ifndef __MOVEMENTDETECTOR_H
#define __MOVEMENTDETECTOR_H

#include "../../Common/include/public.h"
#include "../../Common/include/general.h"

/**
 * @brief 运动目标提取类
 * 提取运动目标以增加神经网络识别精度 
 */
class MovementDetector
{
private:
    vector<vector<vector<float>>> backgroundHistory;
    int historyCount = 0;
    float depthBackground[int(ImageH / _blockSizeH)][int(ImageW / _blockSizeW)] = {0};
    float offsetMap[int(ImageH / _blockSizeH)][int(ImageW / _blockSizeW)] = {0};
    float movementMask[int(ImageH / _blockSizeH)][int(ImageW / _blockSizeW)] = {0};
    Mat tempDepth;
    Mat kernel = getStructuringElement(MORPH_RECT, Size(K_size, K_size));
    vector<Rect> movementTargets;

private:
    void buildBackground();
    vector<Rect> detectMovementTarget(float input[int(ImageH / _blockSizeH)][int(ImageW / _blockSizeW)]);
    Rect rebuildRect(Rect input);

public:
    MovementDetector();
    ~MovementDetector();

    vector<Rect> applyMovementDetector(vector<vector<float>> &input);
};

#endif