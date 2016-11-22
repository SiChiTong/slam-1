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
    double **q;
    double **c;
    double **x;

    Vec2 pt1, pt2;
    ceres::Problem problem;
    ceres::Solver::Options options;
    ceres::Solver::Summary summary;
    ceres::AutoDiffCostFunction<BAResidual, 2, 4, 3, 3> *cost_func;
    BAResidual *r;

    // setup
    q = (double **) malloc(sizeof(double *) * 2);
    c = (double **) malloc(sizeof(double *) * 2);
    x = (double **) malloc(sizeof(double *) * this->x1_pts.rows());

    for (int i = 0; i < 2; i++) {
        q[i] = (double *) malloc(sizeof(double) * 4);
        q[i][0] = 0.0;
        q[i][1] = 0.0;
        q[i][2] = 0.0;
        q[i][3] = 1.0;

        c[i] = (double *) malloc(sizeof(double) * 3);
        c[i][0] = 0.0;
        c[i][1] = 0.0;
        c[i][2] = 40.0;
    }

    for (int i = 0; i < this->x1_pts.rows(); i++) {
        x[i] = (double *) malloc(sizeof(double) * 3);
        x[i][0] = 0.0;
        x[i][1] = 0.0;
        x[i][2] = 0.1;
    }

    // options
    options.max_num_iterations = 10000;
    options.use_nonmonotonic_steps = true;
    options.use_inner_iterations = true;
    options.gradient_tolerance = 1e-16;
    options.function_tolerance = 1e-16;
    options.preconditioner_type = ceres::JACOBI;
    options.linear_solver_type = ceres::SPARSE_SCHUR;
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;

    // image 1
    for (int i = 0; i < this->x1_pts.rows(); i++) {
        pt1 << this->x1_pts(i, 0), this->x1_pts(i, 1);
        r = new BAResidual(this->K, pt1, true);
        cost_func = new ceres::AutoDiffCostFunction<BAResidual, 2, 4, 3, 3>(r);
        problem.AddResidualBlock(cost_func, NULL, q[0], c[0], x[i]);
    }

    // image 2
    for (int i = 0; i < this->x1_pts.rows(); i++) {
        pt2 << this->x2_pts(i, 0), this->x2_pts(i, 1);
        r = new BAResidual(this->K, pt2, false);
        cost_func = new ceres::AutoDiffCostFunction<BAResidual, 2, 4, 3, 3>(r);
        problem.AddResidualBlock(cost_func, NULL, q[1], c[1], x[i]);
    }

    // solve
    ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";
    printf("q: %f %f %f %f\n", q[0][0], q[0][1], q[0][2], q[0][3]);
    printf("c: %f %f %f \n", c[0][0], c[0][1], c[0][2]);
    printf("x: %f %f %f \n\n", x[0][0], x[0][1], x[0][2]);

    printf("q: %f %f %f %f\n", q[1][0], q[1][1], q[1][2], q[1][3]);
    printf("c: %f %f %f \n", c[1][0], c[1][1], c[1][2]);
    printf("x: %f %f %f \n", x[0][0], x[0][1], x[0][2]);

    return 0;
}

}  // end of slam namespace
