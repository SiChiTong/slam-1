#include <functional>

#include <gtest/gtest.h>

#include "slam/optimization/gnm.hpp"
#include "slam/optimization/benchmark.hpp"


static slam::VecX test_function(slam::VecX x)
{
    slam::VecX out(3);

    out << slam::beale(x),
           slam::beale(x),
           slam::beale(x);

    return out;
}

TEST(GNMOpt, constructor)
{
    slam::GNMOpt opt;
    ASSERT_EQ(opt.configured, false);
}

TEST(GNMOpt, configure)
{
    int max_iter;
    int m;
    slam::VecX eta(2);
    slam::VecX x(2);
    slam::GNMOpt opt;

    max_iter = 1000;
    m = 1;
    eta << 1.0, 1.0;
    x << 0.0, 0.0;

    opt.configure(
        max_iter,
        m,
        eta,
        x,
        std::bind(test_function, std::placeholders::_1)
    );

    ASSERT_EQ(opt.configured, true);
    ASSERT_EQ(opt.max_iter, max_iter);
    ASSERT_FLOAT_EQ(opt.eta(0), eta(0));
    ASSERT_FLOAT_EQ(opt.eta(1), eta(1));
    ASSERT_FLOAT_EQ(opt.x(0), x(0));
    ASSERT_FLOAT_EQ(opt.x(1), x(1));
}

TEST(GNMOpt, optimize)
{
    int max_iter;
    int m;
    slam::VecX eta(2);
    slam::VecX x(2);
    slam::GNMOpt opt;

    max_iter = 2;
    m = 3;
    eta << 0.001, 0.001;
    x << 0.5, 1.0;

    opt.configure(
        max_iter,
        m,
        eta,
        x,
        std::bind(test_function, std::placeholders::_1)
    );
    opt.optimize();
    // std::cout << opt.x << std::endl;

    // ASSERT_TRUE(opt.x(0) > 2.7);
    // ASSERT_TRUE(opt.x(0) <= 3.0);
    // ASSERT_TRUE(opt.x(1) > 0.4);
    // ASSERT_TRUE(opt.x(1) <= 0.5);
}

int main(int argc, char* argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
