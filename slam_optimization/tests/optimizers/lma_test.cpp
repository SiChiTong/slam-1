#include <functional>

#include <gtest/gtest.h>

#include "slam/optimization/benchmark.hpp"
#include "slam/optimization/optimizers/lma.hpp"

double test_func(slam::VecX x, slam::VecX beta)
{
    double a;
    double b;
    double y;

    // setup
    a = beta(0);
    b = beta(1);

    // evaluate
    y = a * cos(b * x(0)) + b * sin(a * x(0));

    return y;
}

slam::VecX test_func_jacobian(slam::VecX x, slam::VecX beta)
{
    double a;
    double b;
    slam::VecX J;

    // setup
    a = beta(0);
    b = beta(1);
    J.resize(2);

    // evaluate jacobian
    J(0) = -cos(b * x(0)) - b * cos(a * x(0)) * x(0);
    J(1) = a * sin(b * x(0)) * x(0) - sin(a * x(0));

    return J;
}

std::vector<slam::MatX> generate_data(int nb_samples, double step_size)
{
    slam::MatX x;
    slam::VecX y;
    slam::VecX beta;
    std::vector<slam::MatX> data;

    // setup
    x.resize(nb_samples, 1);
    y.resize(nb_samples);
    beta.resize(2);

    x(0, 0) = 0.0;
    beta(0) = 100.0;
    beta(1) = 105.0;

    // generate data
    for (int i = 0; i < nb_samples; i++) {
        y(i) = test_func(x.block(i, 0, 1, 1), beta);

        if ((i + 1) < nb_samples) {
            x(i + 1, 0) = x(i, 0) + step_size;
        }
    }

    data.push_back(x);
    data.push_back(y);
    data.push_back(beta);

    return data;
}

void test_settings(slam::LMASettings &settings)
{
    std::vector<slam::MatX> data;
    slam::MatX x;

    settings.max_iter = 100;
    settings.lambda = 0.001;
    settings.function = LMA_BIND(test_func);
    settings.jacobian = LMA_BIND(test_func_jacobian);
    settings.nb_inputs = 1;
    settings.nb_params = 2;

    data = generate_data(20, 0.1);
    settings.x = data[0];
    settings.y = data[1];
    settings.beta = data[2];
}

TEST(LMA, constructor)
{
    slam::LMA opt;

    ASSERT_EQ(false, opt.configured);
    ASSERT_EQ(100, opt.max_iter);
    ASSERT_FLOAT_EQ(0.01, opt.lambda);
    ASSERT_EQ(nullptr, opt.function);
    ASSERT_EQ(nullptr, opt.jacobian);
    ASSERT_EQ(0, opt.nb_inputs);
    ASSERT_EQ(0, opt.nb_params);

    ASSERT_FLOAT_EQ(0.0, opt.x(0));
    ASSERT_FLOAT_EQ(0.0, opt.y(0));
    ASSERT_FLOAT_EQ(0.0, opt.beta(0));

    ASSERT_FLOAT_EQ(0.0, opt.y_est(0));
    ASSERT_FLOAT_EQ(0.0, opt.diff(0));

    ASSERT_FLOAT_EQ(0.0, opt.J(0));
    ASSERT_FLOAT_EQ(0.0, opt.H(0));

    ASSERT_EQ(FLT_MAX, opt.error);
}

TEST(LMA, configure)
{
    slam::LMA opt;
    slam::LMASettings settings;

    test_settings(settings);
    opt.configure(settings);

    ASSERT_EQ(true, opt.configured);
    ASSERT_EQ(settings.max_iter, opt.max_iter);
    ASSERT_FLOAT_EQ(settings.lambda, opt.lambda);
    ASSERT_NE(nullptr, opt.function);
    ASSERT_NE(nullptr, opt.jacobian);
    ASSERT_EQ(settings.nb_inputs, opt.nb_inputs);
    ASSERT_EQ(settings.nb_params, opt.nb_params);

    ASSERT_FLOAT_EQ(settings.x(0), opt.x(0));
    ASSERT_FLOAT_EQ(settings.y(0), opt.y(0));
    ASSERT_FLOAT_EQ(settings.beta(0), opt.beta(0));

    ASSERT_FLOAT_EQ(0.0, opt.y_est(0));
    ASSERT_FLOAT_EQ(0.0, opt.diff(0));

    ASSERT_FLOAT_EQ(0.0, opt.J(0, 0));
    ASSERT_FLOAT_EQ(0.0, opt.H(0, 0));

    ASSERT_EQ(FLT_MAX, opt.error);
}

TEST(LMA, evalFunction)
{
    slam::LMA opt;
    slam::LMASettings settings;
    double error;

    // configure
    test_settings(settings);
    opt.configure(settings);

    // test and assert
    opt.evalFunction(opt.beta, error);
    ASSERT_FLOAT_EQ(0.0, error);
}

TEST(LMA, calcGradients)
{
    slam::LMA opt;
    slam::LMASettings settings;
    slam::MatX J_before, H_before;

    // configure
    test_settings(settings);
    opt.configure(settings);
    J_before = opt.J;
    H_before = opt.H;

    // test and assert
    opt.calcGradients(opt.beta);

    std::cout << "J:\n" << opt.J << std::endl << std::endl;
    std::cout << "H:\n" << opt.H << std::endl;

    ASSERT_FALSE(J_before.isApprox(opt.J));
    ASSERT_FALSE(H_before.isApprox(opt.H));
}

TEST(LMA, iterate)
{
    slam::LMA opt;
    slam::LMASettings settings;
    slam::VecX beta_before;
    double error;
    std::vector<slam::VecX> data;

    // configure
    test_settings(settings);
    opt.configure(settings);

    opt.beta << 99, 104;
    beta_before = opt.beta;

    // test and assert
    opt.evalFunction(opt.beta, opt.error);
    opt.calcGradients(opt.beta);

    std::cout << "beta: " << opt.beta.transpose() << std::endl;
    opt.iterate();
    std::cout << "beta: " << opt.beta.transpose() << std::endl;

    ASSERT_FALSE(beta_before.isApprox(opt.beta));
}

TEST(LMA, optimize)
{
    slam::VecX beta;
    slam::LMA opt;
    slam::LMASettings settings;
    std::vector <slam::VecX> data;

    // configure
    test_settings(settings);
    settings.beta << 99, 102;
    opt.configure(settings);

    opt.optimize();
    std::cout << opt.beta.transpose() << std::endl;
}

int main(int argc, char* argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
