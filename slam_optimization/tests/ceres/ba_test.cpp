#include <functional>

#include <gtest/gtest.h>

#include "slam/optimization/ceres/ba.hpp"

#define TEST_DATA_1 "tests/data/ba/pts1.dat"
#define TEST_DATA_2 "tests/data/ba/pts2.dat"


TEST(BAResidual, constructor)
{
    slam::Mat3 K;
    slam::Vec2 x;

    // setup
    K << 1.0, 0.0, 3.0,
         0.0, 2.0, 4.0,
         0.0, 0.0, 1.0;
    x << 130, 62;

    // test and assert
    slam::ceres::BAResidual r(K, x, true);
    ASSERT_FLOAT_EQ(K(0, 0), r.fx);
    ASSERT_FLOAT_EQ(K(1, 1), r.fy);
    ASSERT_FLOAT_EQ(K(0, 2), r.cx);
    ASSERT_FLOAT_EQ(K(1, 2), r.cy);
    ASSERT_FLOAT_EQ(x(0), r.x);
    ASSERT_FLOAT_EQ(x(1), r.y);
    ASSERT_FLOAT_EQ(true, r.origin);
}

TEST(BAResidual, test)
{
    slam::Mat3 K;
    slam::Vec2 p;
    slam::ceres::BAResidual r;

    double q[4];
    double c[3];
    double x[3];
    double e[2];

    // setup
    K << 1.0, 0.0, 0.0,
         0.0, 1.0, 0.0,
         0.0, 0.0, 1.0;
    p << 10, 10;

    q[0] = 0.0;
    q[1] = 0.0;
    q[2] = 0.0;
    q[3] = 0.0;

    c[0] = 0.0;
    c[1] = 0.0;
    c[2] = 0.0;

    x[0] = 10.0;
    x[1] = 10.0;
    x[2] = 1.0;

    r = slam::ceres::BAResidual(K, p, false);
    r(q, c, x, e);
    std::cout << e[0] << std::endl;
    std::cout << e[1] << std::endl;
}

TEST(BundleAdjustment, constructor)
{
    slam::ceres::BundleAdjustment ba;
    ASSERT_EQ(false, ba.configured);
}

TEST(BundleAdjustment, configure)
{
    slam::Mat3 K;
    slam::MatX x1_pts, x2_pts;
    slam::ceres::BundleAdjustment ba;

    // setup
    slam::csv2mat(TEST_DATA_1, false, x1_pts);
    slam::csv2mat(TEST_DATA_2, false, x2_pts);

    // test and assert
    K << 1, 2, 3, 4, 5, 6, 7, 8, 9;
    ba.configure(K, x1_pts, x2_pts);

    ASSERT_EQ(true, ba.configured);
    ASSERT_EQ(x1_pts, ba.x1_pts);
    ASSERT_EQ(x2_pts, ba.x2_pts);
    ASSERT_FLOAT_EQ(K(0, 0), ba.K(0, 0));
    ASSERT_FLOAT_EQ(K(1, 1), ba.K(1, 1));
    ASSERT_FLOAT_EQ(K(2, 2), ba.K(2, 2));
}

TEST(BundleAdjustment, solve)
{
    slam::Mat3 K;
    slam::MatX x1_pts, x2_pts;
    slam::ceres::BundleAdjustment ba;

    // setup
    slam::csv2mat(TEST_DATA_1, false, x1_pts);
    slam::csv2mat(TEST_DATA_2, false, x2_pts);
    // K << 279.0161682343449, 0, 150.3072895826164,
    //      0, 276.3467561622266, 123.3623526538343,
    //      0, 0, 1;
    K << 1.0, 0.0, 0.0,
         0.0, 1.0, 0.0,
         0.0, 0.0, 1.0;
    ba.configure(K, x1_pts, x2_pts);

    // test and assert
    ba.solve();
}

int main(int argc, char* argv[])
{
    google::InitGoogleLogging(argv[0]);
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
