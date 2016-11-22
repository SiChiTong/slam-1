#include "slam/optimization/ba.hpp"


namespace slam {


BundleAdjustment::BundleAdjustment(void)
{
    this->configured = false;
    this->K = MatX::Zero(3, 3);
    this->x1_pts.resize(0, 0);
    this->x2_pts.resize(0, 0);
}

int BundleAdjustment::configure(Mat3 K, MatX x1_pts, MatX x2_pts)
{
    this->configured = true;
    this->K = K;
    this->x1_pts = x1_pts;
    this->x2_pts = x2_pts;

    return 0;
}

int BundleAdjustment::solve(void)
{
    double q[4];
    double c[3];
    double x[3];

    Vec2 pt1, pt2;
    ceres::Problem problem;
    ceres::Solver::Options options;
    ceres::Solver::Summary summary;
    ceres::AutoDiffCostFunction<BAResidual, 2, 4, 3, 3> *cost_func;
    BAResidual *r;

    // setup
    q[0] = 0.0;
    q[1] = 0.0;
    q[2] = 0.0;
    q[3] = 0.0;

    c[0] = 0.0;
    c[1] = 0.0;
    c[2] = 0.0;

    x[0] = 0.0;
    x[1] = 0.0;
    x[2] = 0.1;

    // options
    // options.use_nonmonotonic_steps = true;
    options.preconditioner_type = ceres::SCHUR_JACOBI;
    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.max_num_iterations = 100;

    // load data
    for (int i = 0; i < this->x1_pts.rows(); i++) {
        pt1 << this->x1_pts(i, 0), this->x1_pts(i, 1);
        pt2 << this->x2_pts(i, 0), this->x2_pts(i, 1);

        r = new BAResidual(this->K, pt1, pt2);
        cost_func = new ceres::AutoDiffCostFunction<BAResidual, 2, 4, 3, 3>(r);
        problem.AddResidualBlock(
            cost_func,
            NULL,  // squared loss function
            q,
            c,
            x
        );
    }

    // solve
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.BriefReport() << "\n";
    printf("q: %f %f %f %f\n", q[0], q[1], q[2], q[3]);
    printf("c: %f %f %f \n", c[0], c[1], c[2]);
    printf("x: %f %f %f \n", x[0], x[1], x[2]);

    return 0;
}

}  // end of slam namespace
