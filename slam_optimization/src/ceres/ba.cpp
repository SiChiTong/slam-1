#include "slam/optimization/ceres/ba.hpp"


namespace slam {
namespace ceres {

BundleAdjustment::BundleAdjustment(void)
{
    this->configured = false;

    this->K = MatX::Zero(3, 3);
    this->x1_pts.resize(0, 0);
    this->x2_pts.resize(0, 0);

    this->q = NULL;
    this->c = NULL;
    this->x = NULL;
}

BundleAdjustment::~BundleAdjustment(void)
{
    if (this->configured) {
        for (int i = 0; i < this->x1_pts.rows(); i++) {
            free(this->x[i]);
        }
        free(this->q[0]);
        free(this->q[1]);

        free(this->c[0]);
        free(this->c[1]);

        free(this->x);
        free(this->q);
        free(this->c);
    }
}

int BundleAdjustment::configure(Mat3 K, MatX x1_pts, MatX x2_pts)
{
    Vec3 x1_h;
    Vec3 pt3d_est;

    this->configured = true;

    this->K = K;
    this->x1_pts = x1_pts;
    this->x2_pts = x2_pts;

    this->q = (double **) malloc(sizeof(double *) * 2);
    this->c = (double **) malloc(sizeof(double *) * 2);
    this->x = (double **) malloc(sizeof(double *) * this->x1_pts.rows());

    // initialize quaternion and camera center position
    for (int i = 0; i < 2; i++) {
        // quaternion q = (x, y, z, w)
        this->q[i] = (double *) malloc(sizeof(double) * 4);
        this->q[i][0] = 0.0;
        this->q[i][1] = 0.0;
        this->q[i][2] = 0.0;
        this->q[i][3] = 1.0;

        // camera center c = (x, y, z)
        this->c[i] = (double *) malloc(sizeof(double) * 3);
        this->c[i][0] = 0.0;
        this->c[i][1] = 0.0;
        this->c[i][2] = 0.0;
    }

    // initialize 3D points (x, y, z)
    for (int i = 0; i < this->x1_pts.rows(); i++) {
        this->x[i] = (double *) malloc(sizeof(double) * 3);

        x1_h << this->x1_pts(i, 0), this->x1_pts(i, 1), 1.0;
        pt3d_est = this->K.inverse() * x1_h;

        this->x[i][0] = pt3d_est(0);
        this->x[i][1] = pt3d_est(1);
        this->x[i][2] = pt3d_est(2);
    }

    return 0;
}

int BundleAdjustment::solve(MatX pts3d)
{
    Vec2 pt1, pt2;
    ::ceres::Problem problem;
    ::ceres::Solver::Options options;
    ::ceres::Solver::Summary summary;
    ::ceres::AutoDiffCostFunction<BAResidual, 2, 4, 3, 3> *cost_func;
    BAResidual *r;

    // options
    options.max_num_iterations = 1000;
    options.use_nonmonotonic_steps = true;
    options.preconditioner_type = ::ceres::SCHUR_JACOBI;
    options.linear_solver_type = ::ceres::DENSE_SCHUR;
    // options.minimizer_progress_to_stdout = true;

    ::ceres::LocalParameterization *quat_param;
    quat_param = new ceres::extensions::EigenQuaternionParameterization();

    // image 1
    for (int i = 0; i < this->x1_pts.rows(); i++) {
        pt1 << this->x1_pts(i, 0), this->x1_pts(i, 1);
        r = new BAResidual(this->K, pt1, true);

        cost_func = new ::ceres::AutoDiffCostFunction<
            BAResidual,
            2,
            4,
            3,
            3
        >(r);
        problem.AddResidualBlock(
            cost_func,
            NULL,
            this->q[0],
            this->c[0],
            this->x[i]
        );
    }
    problem.SetParameterization(q[0], quat_param);

    // image 2
    for (int i = 0; i < this->x1_pts.rows(); i++) {
        pt2 << this->x2_pts(i, 0), this->x2_pts(i, 1);
        r = new BAResidual(this->K, pt2, false);

        cost_func = new ::ceres::AutoDiffCostFunction<
            BAResidual,
            2,
            4,
            3,
            3
        >(r);
        problem.AddResidualBlock(
            cost_func,
            NULL,
            this->q[1],
            this->c[1],
            this->x[i]
        );
    }
    problem.SetParameterization(q[1], quat_param);

    // solve
    ::ceres::Solve(options, &problem, &summary);
    std::cout << summary.FullReport() << "\n";

    return 0;
}

}  // end of ceres namespace
}  // end of slam namespace
