#include <iostream>
#include <random>

#include "munit.hpp"
#include "estimation.hpp"
#include "model.hpp"

#define TEST_OUTPUT_FILE "/tmp/estimation_test.output"


// TESTS
int prepareOutputFile(std::ofstream &output_file, std::string output_path);
void recordTimeStep(
    std::ofstream &output_file,
    int i,
    Eigen::Vector3d mea,
    Eigen::Vector3d est
);
int testKalmanFilterInit(void);
int testKalmanFilterEstimate(void);
int testExtendedKalmanFilterInit(void);


class MockRobot
{
public:

    VecXd gFunc(VecXd, VecXd, float)
    {
        std::cout << "gFunc" << std::endl;
        Eigen::VectorXd x;
        return x;
    }

    MatXd GFunc(VecXd, VecXd, float)
    {
        std::cout << "GFunc" << std::endl;
        return Eigen::MatrixXd::Identity(3, 3);
    }

    VecXd hFunc(VecXd)
    {
        Eigen::VectorXd x;
        std::cout << "hFunc" << std::endl;
        return x;
    }

    MatXd HFunc(VecXd)
    {
        std::cout << "HFunc" << std::endl;
        return Eigen::MatrixXd::Identity(3, 3);
    }
};

int prepareOutputFile(std::ofstream &output_file, std::string output_path)
{
    output_file.open(output_path);

    output_file << "time_step" << ",";

    output_file << "x" << ",";
    output_file << "y" << ",";
    output_file << "z" << ",";

    output_file << "bx" << ",";
    output_file << "by" << ",";
    output_file << "bz" << std::endl;

    return 0;
}

void recordTimeStep(
    std::ofstream &output_file,
    int i,
    Eigen::Vector3d mea,
    Eigen::Vector3d est
)
{
    // record true state x, y, z
    output_file << i << ",";
    output_file << mea(0) << ",";
    output_file << mea(1) << ",";
    output_file << mea(2) << ",";

    // record belief state x, y, z
    output_file << est(0) << ",";
    output_file << est(1) << ",";
    output_file << est(2) << std::endl;
}

int testKalmanFilterInit(void)
{
    KalmanFilter estimator;
    Eigen::VectorXd mu(9);
    Eigen::MatrixXd R(9, 9);
    Eigen::MatrixXd C(3, 9);
    Eigen::MatrixXd Q(3, 3);

    mu << 0, 0, 0, 0, 0, 0, 0, 0, 0;
    R << 0.5, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0.5, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0.5, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 1.0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 1.0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 1.0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 1.0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 1.0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 1.0;
    C << 1, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 1, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 1, 0, 0, 0, 0, 0, 0;
    Q << 20, 0, 0,
         0, 20, 0,
         0, 0, 20;
    estimator.init(mu, R, C, Q);

    return 0;
}

int testKalmanFilterEstimate(void)
{
    float dt;

    KalmanFilter estimator;
    Eigen::Vector3d pos;
    Eigen::Vector3d vel;
    Eigen::Vector3d acc;
    Eigen::VectorXd state(9);
    Eigen::VectorXd mu(9);
    Eigen::MatrixXd A(9, 9);
    Eigen::MatrixXd R(9, 9);
    Eigen::MatrixXd C(3, 9);
    Eigen::MatrixXd Q(3, 3);
    Eigen::VectorXd y(3);
    Eigen::VectorXd motion_noise(3);
    Eigen::Vector3d mea;
    Eigen::Vector3d est;
    std::ofstream output_file;
    std::default_random_engine generator;
    std::normal_distribution<float> norm_dist_x(0, 0.5);
    std::normal_distribution<float> norm_dist_y(0, 0.5);
    std::normal_distribution<float> norm_dist_z(0, 0.5);

    // setup
    dt = 0.1;
    pos << 0, 0, 0;
    vel << 9, 30, 0;
    acc << 0, -10, 0;
    mu << 0.0, 0.0, 0.0,    // x, y, z
          9.0, 30.0, 0.0,   // x_dot, y_dot, z_dot
          0.0, -10.0, 0.0;  // x_ddot, y_ddot, z_ddot
    R << 0.5, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0.5, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0.5, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 1.0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 1.0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 1.0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 1.0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 1.0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 1.0;
    C << 1, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 1, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 1, 0, 0, 0, 0, 0, 0;
    Q << 20, 0, 0,
         0, 20, 0,
         0, 0, 20;
    estimator.init(mu, R, C, Q);
    prepareOutputFile(output_file, TEST_OUTPUT_FILE);

    // estimate
    for (int i = 0; i < 20; i++) {
        // update true state
        vel = vel + acc * dt;
        pos = pos + vel * dt;
        state << pos(0), pos(1), pos(2),
                 vel(0), vel(1), vel(2),
                 acc(0), acc(1), acc(2);

        // perform measurement
        motion_noise << norm_dist_x(generator),
                        norm_dist_y(generator),
                        norm_dist_z(generator);
        y = estimator.C * state + motion_noise;

        // estimate
        A <<
            1.0, 0, 0, dt, 0, 0, pow(dt, 2) / 2.0, 0, 0,
            0, 1.0, 0, 0, dt, 0, 0, pow(dt, 2) / 2.0, 0,
            0, 0, 1.0, 0, 0, dt, 0, 0, pow(dt, 2) / 2.0,
            0, 0, 0, 1.0, 0, 0, dt, 0, 0,
            0, 0, 0, 0, 1.0, 0, 0, dt, 0,
            0, 0, 0, 0, 0, 1.0, 0, 0, dt,
            0, 0, 0, 0, 0, 0, 1.0, 0, 0,
            0, 0, 0, 0, 0, 0, 0, 1.0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 1.0;
        estimator.estimate(A, y);

        // record
        mea << pos(0), pos(1), pos(2);
        est << estimator.mu(0), estimator.mu(1), estimator.mu(2);
        recordTimeStep(output_file, i, mea, est);
    }
    output_file.close();

    return 0;
}

int testExtendedKalmanFilterInit(void)
{
    Eigen::VectorXd mu(9);
    Eigen::MatrixXd R(9, 9);
    Eigen::MatrixXd Q(3, 3);
    TwoWheelRobot robot;
    ExtendedKalmanFilter estimator;

    mu << 0, 0, 0, 0, 0, 0, 0, 0, 0;
    R << 0.5, 0, 0, 0, 0, 0, 0, 0, 0,
         0, 0.5, 0, 0, 0, 0, 0, 0, 0,
         0, 0, 0.5, 0, 0, 0, 0, 0, 0,
         0, 0, 0, 1.0, 0, 0, 0, 0, 0,
         0, 0, 0, 0, 1.0, 0, 0, 0, 0,
         0, 0, 0, 0, 0, 1.0, 0, 0, 0,
         0, 0, 0, 0, 0, 0, 1.0, 0, 0,
         0, 0, 0, 0, 0, 0, 0, 1.0, 0,
         0, 0, 0, 0, 0, 0, 0, 0, 1.0;
    Q << 20, 0, 0,
         0, 20, 0,
         0, 0, 20;
    estimator.init(mu, R, Q);

    return 0;
}

void test_suite(void)
{
    mu_add_test(testKalmanFilterInit);
    mu_add_test(testKalmanFilterEstimate);
    mu_add_test(testExtendedKalmanFilterInit);
}

mu_run_tests(test_suite)
