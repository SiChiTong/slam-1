#include <functional>

#include <gtest/gtest.h>

#include "slam/optimization/nm.hpp"
#include "slam/optimization/benchmark.hpp"

double test_function(slam::VecX x)
{
    return pow(x(0), 2) + pow(x(1), 2);
}

TEST(NMOpt, constructor)
{
    slam::NMOpt opt;
    ASSERT_EQ(opt.configured, false);
}

TEST(NMOpt, configure)
{
    int max_iter;
    int nb_funcs;
    int nb_params;
    slam::VecX eta(2);
    slam::VecX x(2);
    slam::NMOpt opt;

    max_iter = 1000;
    nb_funcs = 1;
    nb_params = 2;
    eta << 1.0, 1.0;
    x << 0.0, 0.0;

    opt.configure(
        max_iter,
        eta,
        x,
        std::bind(slam::beale, std::placeholders::_1)
    );

    ASSERT_EQ(opt.configured, true);
    ASSERT_EQ(opt.max_iter, max_iter);
    ASSERT_FLOAT_EQ(opt.eta(0), eta(0));
    ASSERT_FLOAT_EQ(opt.eta(1), eta(1));
    ASSERT_FLOAT_EQ(opt.x(0), x(0));
    ASSERT_FLOAT_EQ(opt.x(1), x(1));
}

TEST(NMOpt, calcGradient)
{
    int max_iter;
    slam::VecX eta(2);
    slam::VecX x(2);
    slam::VecX df(2, 1);
    slam::NMOpt opt;

    max_iter = 1000;
    eta << 1.0, 1.0;
    x << 0.0, 0.0;

    opt.configure(
        max_iter,
        eta,
        x,
        std::bind(test_function, std::placeholders::_1)
    );
    opt.calcGradient(df);
    // std::cout << df << std::endl;

    ASSERT_FLOAT_EQ(0.0, df(0));
    ASSERT_FLOAT_EQ(0.0, df(1));
}

TEST(NMOpt, calcHessian)
{
    int max_iter;
    slam::VecX eta(2);
    slam::VecX x(2);
    slam::MatX H(2, 2);
    slam::NMOpt opt;

    max_iter = 1000;
    eta << 1.0, 1.0;
    x << 0.0, 0.0;

    opt.configure(
        max_iter,
        eta,
        x,
        std::bind(test_function, std::placeholders::_1)
    );
    opt.calcHessian(H);
    // std::cout << H << std::endl;
}

TEST(NMOpt, optimize)
{
    int max_iter;
    slam::VecX eta(2);
    slam::VecX x(2);
    slam::NMOpt opt;

    max_iter = 100;
    eta << 0.1, 0.1;
    x << 2.0, 2.0;

    opt.configure(
        max_iter,
        eta,
        x,
        std::bind(test_function, std::placeholders::_1)
    );
    opt.optimize();
    // std::cout << opt.x << std::endl;

    ASSERT_TRUE(opt.x(0) > -0.1);
    ASSERT_TRUE(opt.x(0) < 0.1);
    ASSERT_TRUE(opt.x(1) > -0.1);
    ASSERT_TRUE(opt.x(1) < 0.1);
}

int main(int argc, char* argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
