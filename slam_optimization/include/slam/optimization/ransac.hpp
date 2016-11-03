#ifndef __SLAM_OPTIMIZATION_RANSAC_HPP__
#define __SLAM_OPTIMIZATION_RANSAC_HPP__

#include <iostream>
#include <math.h>

#include <Eigen/Dense>


namespace slam {
namespace optimization {

class RANSACParams
{
public:
    bool configured;

    int min_pts;
    int max_iter;
    float thresh_dist;
    float inlier_ratio;

    RANSACParams(void);
    int configure(void);
};

class RANSAC
{
public:
    bool configured;

    int max_iter;
    double thresh_ratio;
    double thresh_dist;

    int iter;
    std::vector<int> inliers;
    double threshold;
    int max_inliers;
    double model_params[2];

    RANSAC(void);
    int configure(int max_iter, double threshold_ratio, double threshold_dist);
    int randomSample(Eigen::MatrixXd &data, Eigen::Vector2d &sample);
    int computeDistances(
        Eigen::MatrixXd &data,
        Eigen::Vector2d &p1,
        Eigen::Vector2d &p2,
        Eigen::VectorXd &dists
    );
    int computeInliers(Eigen::VectorXd &dists);
    int update(Eigen::Vector2d &p1, Eigen::Vector2d &p2);
    int printStats(void);
    int optimize(Eigen::MatrixXd &data);
};

}  // end of slam namespace
}  // end of slam namespace
#endif
