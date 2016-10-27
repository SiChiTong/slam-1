#include <iostream>
#include <random>

#include "slam/utils/munit.hpp"
#include "slam/utils/utils.hpp"
#include "slam/kinematics/model.hpp"
#include "slam/estimation/estimation.hpp"

#define TEST_KF_OUTPUT_FILE "/tmp/estimation_kf_test.output"
#define TEST_EKF_OUTPUT_FILE "/tmp/estimation_ekf_test.output"
#define TEST_PF_OUTPUT_FILE "/tmp/estimation_pf_test.output"


// TESTS
int prepareOutputFile(std::ofstream &output_file, std::string output_path);
void recordTimeStep(
    std::ofstream &output_file,
    int i,
    slam::Vec3 mea,
    slam::Vec3 est
);
int testKalmanFilter(void);
int testExtendedKalmanFilter(void);
int testParticleFilter(void);


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
    slam::Vec3 mea,
    slam::Vec3 est
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

int testKalmanFilter(void)
{
    float dt;

    slam::KalmanFilter kf;
    slam::Vec3 pos;
    slam::Vec3 vel;
    slam::Vec3 acc;
    slam::VecX state(9);
    slam::VecX mu(9);
    slam::MatX A(9, 9);
    slam::MatX R(9, 9);
    slam::MatX C(3, 9);
    slam::MatX Q(3, 3);
    slam::VecX y(3);
    slam::VecX motion_noise(3);
    slam::Vec3 mea;
    slam::Vec3 est;
    std::ofstream output_file;
    std::default_random_engine rgen;
    std::normal_distribution<float> norm_x(0, 0.5);
    std::normal_distribution<float> norm_y(0, 0.5);
    std::normal_distribution<float> norm_z(0, 0.5);

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
    kf.init(mu, R, C, Q);
    prepareOutputFile(output_file, TEST_KF_OUTPUT_FILE);

    // estimate
    for (int i = 0; i < 20; i++) {
        // update true state
        vel = vel + acc * dt;
        pos = pos + vel * dt;
        state << pos(0), pos(1), pos(2),
                 vel(0), vel(1), vel(2),
                 acc(0), acc(1), acc(2);

        // perform measurement
        motion_noise << norm_x(rgen), norm_y(rgen), norm_z(rgen);
        y = kf.C * state + motion_noise;

        // estimate
        A << 1.0, 0, 0, dt, 0, 0, pow(dt, 2) / 2.0, 0, 0,
             0, 1.0, 0, 0, dt, 0, 0, pow(dt, 2) / 2.0, 0,
             0, 0, 1.0, 0, 0, dt, 0, 0, pow(dt, 2) / 2.0,
             0, 0, 0, 1.0, 0, 0, dt, 0, 0,
             0, 0, 0, 0, 1.0, 0, 0, dt, 0,
             0, 0, 0, 0, 0, 1.0, 0, 0, dt,
             0, 0, 0, 0, 0, 0, 1.0, 0, 0,
             0, 0, 0, 0, 0, 0, 0, 1.0, 0,
             0, 0, 0, 0, 0, 0, 0, 0, 1.0;
        kf.estimate(A, y);

        // record
        mea << pos(0), pos(1), pos(2);
        est << kf.mu(0), kf.mu(1), kf.mu(2);
        recordTimeStep(output_file, i, mea, est);
    }
    output_file.close();

    return 0;
}

int testExtendedKalmanFilter(void)
{
    float dt;
    slam::VecX x(3);
    slam::VecX y(3);
    slam::VecX gaussian_noise(3);
    slam::VecX mu(3);
    slam::VecX u(3);
    slam::MatX R(3, 3);
    slam::MatX Q(3, 3);
    slam::VecX g(3);
    slam::MatX G(3, 3);
    slam::VecX h(3);
    slam::MatX H(3, 3);
    slam::ExtendedKalmanFilter ekf;
    std::ofstream output_file;
    std::default_random_engine rgen;
    std::normal_distribution<float> norm_x(0, pow(0.5, 2));
    std::normal_distribution<float> norm_y(0, pow(0.5, 2));
    std::normal_distribution<float> norm_theta(0, pow(slam::deg2rad(0.5), 2));

    // setup
    dt = 0.01;
    x << 0, 0, 0;
    mu << 0, 0, 0;
    R << pow(0.05, 2), 0, 0,
         0, pow(0.05, 2), 0,
         0, 0, pow(slam::deg2rad(0.5), 2);
    Q << pow(0.5, 2), 0, 0,
         0, pow(0.5, 2), 0,
         0, 0, pow(slam::deg2rad(10), 2);
    u << -15.5, -10.5, 1.5;
    ekf.init(mu, R, Q);
    prepareOutputFile(output_file, TEST_EKF_OUTPUT_FILE);

    // loop
    for (int i = 0; i < 100; i++) {
        // update true state
        x << x(0) + u(0) * cos(x(2)) * dt,
             x(1) + u(0) * sin(x(2)) * dt,
             x(2) + u(1) * dt;

        // take measurement
        gaussian_noise << norm_x(rgen), norm_y(rgen), norm_theta(rgen);
        y = x + gaussian_noise;

        // propagate motion model
        g << ekf.mu(0) + u(0) * cos(ekf.mu(2)) * dt,
            ekf.mu(1) + u(0) * sin(ekf.mu(2)) * dt,
            ekf.mu(2) + u(1) * dt;
        G << 1, 0, (-u(0) * sin(ekf.mu(2)) * dt),
            0, 1, (u(0) * cos(ekf.mu(2)) * dt),
            0, 0, 1;
        ekf.predictionUpdate(g, G);

        // propagate measurement
        H = Eigen::MatrixXf::Identity(3, 3);
        h = H * ekf.mu;
        ekf.measurementUpdate(h, H, y);

        // record
        recordTimeStep(output_file, i, x, ekf.mu);
    }
    output_file.close();

    return 0;
}

// static VecX update_state(VecX x, VecX u, MatX R, float dt)
// {
//     VecX x_p;
//     VecX gaussian_noise(3);
//     std::default_random_engine rgen;
//     std::normal_distribution<float> norm_x(0, pow(R(0, 0), 2));
//     std::normal_distribution<float> norm_y(0, pow(R(1, 1), 2));
//     std::normal_distribution<float> norm_theta(0, pow(R(2, 2), 2));
//
//     gaussian_noise << norm_x(rgen), norm_y(rgen), norm_theta(rgen);
//     x_p << x(0) + u(0) * cos(x(2)) * dt,
//            x(1) + u(0) * sin(x(2)) * dt,
//            x(2) + u(1) * dt;
//     x_p += gaussian_noise;
//
//     return x_p;
// }
//
// static VecX create_hypothesis(VecX x_p)
// {
//     MatX I;
//     VecX hx_p;
//
//     I = Eigen::MatrixXf::Identity(3, 3);
//     hx_p = I * x_p;
//
//     return hx_p;
// }

// int testParticleFilterUpdate(void)
// {
//     float dt;
//     VecX x(3);
//     VecX mu(3);
//     ParticleFilter pf;
//     std::ofstream output_file;
//
//     // setup
//     dt = 0.01;
//     x << 0, 0, 0;
//     mu << 0, 0, 0;
//     pf.init(100, mu);
//     prepareOutputFile(output_file, TEST_PF_OUTPUT_FILE);
//
//     // loop
//     for (int i = 0; i < 100; i++) {
//
//
//
//     }
//     output_file.close();
//
//     return 0;
// }

void test_suite(void)
{
    mu_add_test(testKalmanFilter);
    mu_add_test(testExtendedKalmanFilter);
    // mu_add_test(testParticleFilterUpdate);
}

mu_run_tests(test_suite)
