#ifndef __SORT_H
#define __SORT_H

#include "../../Common/include/public.h"
#include "datatype.h"

class NearestNeighborDistanceMetric
{
public:
    enum METRIC_TYPE
    {
        euclidean = 1,
        cosine
    };
    DYNAMICM distance(const FEATURESS &features, const std::vector<int> &targets);
    void partial_fit(std::vector<TRACKER_DATA> &tid_feats, std::vector<int> &active_targets);
    float mating_threshold;

public:
    NearestNeighborDistanceMetric(
        NearestNeighborDistanceMetric::METRIC_TYPE metric,
        float matching_threshold, int budget);
    ~NearestNeighborDistanceMetric();

private:
    typedef Eigen::VectorXf (NearestNeighborDistanceMetric::*PTRFUN)(const FEATURESS &, const FEATURESS &);
    Eigen::VectorXf _nncosine_distance(const FEATURESS &x, const FEATURESS &y);
    Eigen::VectorXf _nneuclidean_distance(const FEATURESS &x, const FEATURESS &y);

    Eigen::MatrixXf _pdist(const FEATURESS &x, const FEATURESS &y);
    Eigen::MatrixXf _cosine_distance(const FEATURESS &a, const FEATURESS &b, bool data_is_normalized = false);
    PTRFUN _metric;
    int budget;
    std::map<int, FEATURESS> samples;
};

#endif