#ifndef DATATYPE_H
#define DATATYPE_H

#include <cstddef>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>

typedef struct CLSCONF
{
    CLSCONF()
    {
        this->cls = -1;
        this->conf = -1;
    }
    CLSCONF(int cls, float conf)
    {
        this->cls = cls;
        this->conf = conf;
    }
    int cls;
    float conf;
} CLSCONF;

typedef Eigen::Matrix<float, 1, 4, Eigen::RowMajor> DETECTBOX;
typedef Eigen::Matrix<float, -1, 4, Eigen::RowMajor> DETECTBOXSS;
typedef Eigen::Matrix<float, 1, 256, Eigen::RowMajor> FEATURE;
typedef Eigen::Matrix<float, Eigen::Dynamic, 256, Eigen::RowMajor> FEATURESS;
// typedef std::vector<FEATURE> FEATURESS;

// Kalmanfilter
// typedef Eigen::Matrix<float, 8, 8, Eigen::RowMajor> KAL_FILTER;
typedef Eigen::Matrix<float, 1, 8, Eigen::RowMajor> KAL_MEAN;
typedef Eigen::Matrix<float, 8, 8, Eigen::RowMajor> KAL_COVA;
typedef Eigen::Matrix<float, 1, 4, Eigen::RowMajor> KAL_HMEAN;
typedef Eigen::Matrix<float, 4, 4, Eigen::RowMajor> KAL_HCOVA;
using KAL_DATA = std::pair<KAL_MEAN, KAL_COVA>;
using KAL_HDATA = std::pair<KAL_HMEAN, KAL_HCOVA>;

// main
using RESULT_DATA = std::pair<int, DETECTBOX>;

// tracker:
using TRACKER_DATA = std::pair<int, FEATURESS>;
using MATCH_DATA = std::pair<int, int>;
typedef struct t
{
    std::vector<MATCH_DATA> matches;
    std::vector<int> unmatched_tracks;
    std::vector<int> unmatched_detections;
} TRACHER_MATCHD;

// linear_assignment:
typedef Eigen::Matrix<float, -1, -1, Eigen::RowMajor> DYNAMICM;

enum DETECTBOX_IDX
{
    IDX_X = 0,
    IDX_Y,
    IDX_W,
    IDX_H
};

class DETECTION_ROW
{
private:
    const float kRatio = 0.5;

public:
    DETECTBOX tlwh;
    float confidence;
    FEATURE feature;
    DETECTBOX to_xyah() const
    {
        //(centerx, centery, ration, h)
        DETECTBOX ret = tlwh;
        ret(0, IDX_X) += (ret(0, IDX_W) * kRatio);
        ret(0, IDX_Y) += (ret(0, IDX_H) * kRatio);
        ret(0, IDX_W) /= ret(0, IDX_H);
        return ret;
    }
    DETECTBOX to_tlbr() const
    {
        //(x,y,xx,yy)
        DETECTBOX ret = tlwh;
        ret(0, IDX_X) += ret(0, IDX_W);
        ret(0, IDX_Y) += ret(0, IDX_H);
        return ret;
    }
};

typedef std::vector<DETECTION_ROW> DETECTIONS;
typedef std::pair<std::vector<CLSCONF>, DETECTIONS> DETECTIONSV2;

#endif