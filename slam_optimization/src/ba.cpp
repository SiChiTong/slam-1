#include "slam/optimization/ba.hpp"


namespace slam {


BundleAdjustment::BundleAdjustment(void)
{
    this->configured = false;
    // this->x1_pts.clear();
    // this->x2_pts.clear();
}

// int BundleAdjustment::configure(
//     std::vector<cv::Point2f> x1_pts,
//     std::vector<cv::Point2f> x2_pts
// )
// {
//     this->configured = true;
//     this->x1_pts = x1_pts;
//     this->x2_pts = x2_pts;
//
//     return 0;
// }

int BundleAdjustment::solve(void)
{
    double q[4];
    Vec2 data_point;
    ceres::Problem problem;
    ceres::Solver::Options options;
    ceres::Solver::Summary summary;
    ceres::AutoDiffCostFunction<BAResidual, 1, 1, 1> *cost_func;
    BAResidual *res;

    // options
    options.max_num_iterations = 25;
    // options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;

    // // load data
    // for (int i = 0; i < kNumObservations; ++i) {
    //     data_point << data[2 * i], data[2 * i + 1];
    //     res = new BAResidual(data_point);
    //     cost_func = new ceres::AutoDiffCostFunction<BAResidual, 1, 1, 1>(res);
    //     problem.AddResidualBlock(cost_func, NULL, &pose);
    // }

    // // solve
    // ceres::Solve(options, &problem, &summary);
    // std::cout << summary.BriefReport() << "\n";
    // std::cout << "Initial m: " << 0.0 << " c: " << 0.0 << "\n";
    // std::cout << "Final   m: " << m << " c: " << c << "\n";

    return 0;
}

}  // end of slam namespace
