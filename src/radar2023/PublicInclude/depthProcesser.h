#ifndef __DEPTHPROCESSER_H
#define __DEPTHPROCESSER_H

#include "./public.h"

/**
 * @brief 深度信息处理类
 * 处理雷达点云信息为相机对应的深度图
 */
class DepthQueue
{
private:
    bool _initflag = false;
    queue<Matrix<int, 2, MaxPointsNum>> processQueue;
    Matrix<float, 3, 3> K_0;
    Matrix<float, 1, 5> C_0;
    Matrix<float, 4, 4> E_0;
    Matrix<float, 1, 3> tvec;
    Matrix<float, 1, 3> rvec;
    Mat rvec_Mat, E_0_Mat;
    vector<vector<float>> depth;

public:
    DepthQueue(Matrix<float, 3, 3> &K_0, Matrix<float, 1, 5> &C_0, Matrix<float, 4, 4> &E_0);
    ~DepthQueue();

    int getProcessQueueSize() { return processQueue.size(); };

    vector<float> detectDepth(vector<ArmorBoundingBox> &armorBoundingBoxs);

    vector<vector<float>> pushback(pcl::PointCloud<pcl::PointXYZ> &pc);
};

#endif