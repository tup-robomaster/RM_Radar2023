#ifndef __DEPTHPROCESSER_H
#define __DEPTHPROCESSER_H

#include "../../Common/include/public.h"

/**
 * @brief 深度信息处理类
 * 处理雷达点云信息为相机对应的深度图
 */
class DepthQueue
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    typedef std::shared_ptr<DepthQueue> Ptr;

private:
    bool _initflag = false;
    queue<Matrix<int, 2, MaxPointsNum>> processQueue;
    Matrix<float, 3, 3> K_0;
    Matrix<float, 1, 5> C_0;
    Matrix<float, 4, 4> E_0;
    vector<vector<float>> depth;

public:
    DepthQueue();
    DepthQueue(Matrix<float, 3, 3> &K_0, Matrix<float, 1, 5> &C_0, Matrix<float, 4, 4> &E_0);
    ~DepthQueue();

    int getProcessQueueSize() { return processQueue.size(); };

    vector<vector<float>> pushback(pcl::PointCloud<pcl::PointXYZ> &pc);
};

#endif