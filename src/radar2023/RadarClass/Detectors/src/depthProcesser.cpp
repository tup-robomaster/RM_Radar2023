#include "../include/depthProcesser.h"

DepthQueue::DepthQueue()
{
}

DepthQueue::DepthQueue(Matrix<float, 3, 3> &K_0, Matrix<float, 1, 5> &C_0, Matrix<float, 4, 4> &E_0)
{
    this->K_0 = K_0;
    this->C_0 = C_0;
    this->E_0 = E_0;
    this->tvec = E_0.block(0, 3, 3, 1).transpose();
    eigen2cv(Matrix<float, 3, 3>(E_0.block(0, 0, 3, 3)), this->E_0_Mat);
    Rodrigues(this->E_0_Mat, this->rvec_Mat);
    cv2eigen(this->rvec_Mat, this->rvec);
    vector<float> tmp(ImageW, 0.);
    this->depth.resize(ImageH, tmp);
}

DepthQueue::~DepthQueue()
{
}

vector<vector<float>> DepthQueue::pushback(pcl::PointCloud<pcl::PointXYZ> &pc)
{
    if (this->processQueue.empty())
    {
        this->_initflag = true;
    }
    Matrix4Xf pc_Matrix = pc.getMatrixXfMap();
    int cols = pc_Matrix.cols();
    Matrix3Xf transformed_points = (this->E_0 * pc_Matrix).block(0, 0, 3, cols);
    Matrix<float, 3, MaxPointsNum> pointsBox;
    Matrix<float, 1, MaxPointsNum> dptBox;
    Matrix<int, 2, MaxPointsNum> ipBox;
    pointsBox.leftCols(cols) << transformed_points;
    dptBox.leftCols(cols) << transformed_points.row(2);
    ipBox << ((this->K_0 * pointsBox).array().rowwise() * (pointsBox.row(2).array().inverse())).topRows(2).matrix().cast<int>();
    auto inside_x = (ipBox.row(0).array() >= 0 && ipBox.row(0).array() < ImageW);
    auto inside_y = (ipBox.row(1).array() >= 0 && ipBox.row(1).array() < ImageH);
    ipBox.row(0) << (inside_x).select(ipBox.row(0), MatrixXf::Constant(1, MaxPointsNum, 0));
    ipBox.row(1) << (inside_y).select(ipBox.row(1), MatrixXf::Constant(1, MaxPointsNum, 0));
    this->processQueue.push(ipBox);
    if (this->processQueue.size() > maxQueueSize)
    {
        Matrix<int, 2, MaxPointsNum> outpoints = this->processQueue.front();
        for (int i = 0; i < MaxPointsNum; i++)
        {
            if (this->depth[outpoints(1, i)][outpoints(0, i)] == 0.)
                continue;
            this->depth[outpoints(1, i)][outpoints(0, i)] = 0.;
        }
        this->processQueue.pop();
    }
    for (int i = 0; i < MaxPointsNum; i++)
    {
        if (dptBox(0, i) > 0)
        {
            if ((this->depth[ipBox(1, i)][ipBox(0, i)] != 0. && dptBox(0, i) < this->depth[ipBox(1, i)][ipBox(0, i)]) || this->depth[ipBox(1, i)][ipBox(0, i)] == 0.)
                this->depth[ipBox(1, i)][ipBox(0, i)] = dptBox(0, i);
        }
        else
            break;
    }
    return this->depth;
}

vector<float> DepthQueue::detectDepth(vector<ArmorBoundingBox> &armorBoundingBoxs)
{
    vector<float> depthBox;
    if (armorBoundingBoxs.size() == 0)
        return depthBox;
    for (const auto &it : armorBoundingBoxs)
    {
        int count = 0;
        vector<float> tempBox;
        float center[2] = {it.x0 + it.w / 2, it.y0 + it.h / 2};
        for (int i = int(max<float>(center[1] - it.h, 0.)); i < int(min<float>(center[1] + it.h, ImageH)); ++i)
        {
            for (int j = int(max<float>(center[0] - it.w, 0.)); j < int(min<float>(center[0] + it.w, ImageW)); ++j)
            {
                if (this->depth[i][j] == 0.)
                    continue;
                tempBox.emplace_back(this->depth[i][j]);
                ++count;
            }
        }
        int tempNum = 0;
        for (const auto &jt : tempBox)
        {
            tempNum += jt;
        }
        depthBox.emplace_back(tempNum / count);
    }
    return depthBox;
}