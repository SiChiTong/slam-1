#include "slam/optimization/ba.hpp"


namespace slam {

BundleAdjustment::BundleAdjustment(void)
{
    this->configured = false;

}

int BundleAdjustment::configure(void)
{
    this->configured = true;

    return 0;
}

int BundleAdjustment::solve(void)
{
    double x;
    ceres::Problem problem;
    ceres::CostFunction *cost_function;
    ceres::Solver::Options options;
    ceres::Solver::Summary summary;

    // setup
    cost_function = new ceres::AutoDiffCostFunction<
        CostFunctor,
        1,
        1
    >(new CostFunctor);

    x = 0.5;
    options.minimizer_progress_to_stdout = true;
    problem.AddResidualBlock(cost_function, NULL, &x);

    // solve
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << "\n";
    std::cout << "x : " << x << "\n";

    return 0;
}

}  // end of slam namespace
