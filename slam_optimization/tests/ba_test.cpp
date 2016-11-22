#include <functional>

#include <gtest/gtest.h>

#include "slam/optimization/ba.hpp"

#define TEST_DATA_1 "tests/data/ba/pts1.dat"
#define TEST_DATA_2 "tests/data/ba/pts2.dat"


TEST(BAResidual, constructor)
{
    slam::Mat3 K;
    slam::Vec2 x1;
    slam::Vec2 x2;

    // setup
    K << 1.0, 0.0, 3.0,
         0.0, 2.0, 4.0,
         0.0, 0.0, 1.0;
    x1 << 130, 62;
    x2 << 131.474, 73.2267;

    // test and assert
    slam::BAResidual r(K, x1, x2);
    ASSERT_FLOAT_EQ(K(0, 0), r.fx);
    ASSERT_FLOAT_EQ(K(1, 1), r.fy);
    ASSERT_FLOAT_EQ(K(0, 2), r.cx);
    ASSERT_FLOAT_EQ(K(1, 2), r.cy);
    ASSERT_FLOAT_EQ(x1(0), r.x1_x);
    ASSERT_FLOAT_EQ(x1(1), r.x1_y);
    ASSERT_FLOAT_EQ(x2(0), r.x2_x);
    ASSERT_FLOAT_EQ(x2(1), r.x2_y);
}

// TEST(BAResidual, setupRotationMatrix)
// {
//     slam::Mat3 R;
//     double q[4];
//     slam::BAResidual r;
//
//     // setup
//     q[0] = 0.0;
//     q[1] = 0.0;
//     q[2] = 0.0;
//     q[3] = 0.0;
//
//     // test and assert
//     r.setupRotationMatrix(q, R);
//     ASSERT_EQ(1.0, R(0, 0));
//     ASSERT_EQ(1.0, R(1, 1));
//     ASSERT_EQ(1.0, R(2, 2));
// }
//
// TEST(BAResidual, setupCameraCenter)
// {
//     double c[3];
//     slam::Vec3 C;
//     slam::BAResidual r;
//
//     // setup
//     c[0] = 1;
//     c[1] = 2;
//     c[2] = 3;
//
//     // test and assert
//     r.setupCameraCenter(c, C);
//     ASSERT_EQ(1.0, C(0));
//     ASSERT_EQ(2.0, C(1));
//     ASSERT_EQ(3.0, C(2));
// }
//
// TEST(BAResidual, setup3DPoint)
// {
//     double x[3];
//     slam::Vec3 X;
//     slam::BAResidual r;
//
//     // setup
//     x[0] = 1;
//     x[1] = 2;
//     x[2] = 3;
//
//     // test and assert
//     r.setup3DPoint(x, X);
//     ASSERT_EQ(1.0, X(0));
//     ASSERT_EQ(2.0, X(1));
//     ASSERT_EQ(3.0, X(2));
// }
//
// TEST(BAResidual, calcReprojectionError)
// {
//     slam::Mat3 K;
//     slam::Mat3 R;
//     slam::Vec3 C;
//     slam::Vec3 X;
//     slam::Vec2 m_tilde;
//     slam::BAResidual r;
//     double residual;
//
//     // setup
//     K << 1, 0, 0,
//          0, 1, 0,
//          0, 0, 1;
//
//     R << 1, 0, 0,
//          0, 1, 0,
//          0, 0, 1;
//
//     C << 0, 1, 0;
//
//     X << 0, 0, 1;
//
//     m_tilde << 0, 0;
//
//     // test and assert
//     r.K = K;
//     residual = r.calcReprojectionError(R, C, X, m_tilde);
//     std::cout << residual << std::endl;
// }
//
// TEST(BAResidual, test)
// {
//     slam::Mat3 K;
//     slam::Vec2 x1;
//     slam::Vec2 x2;
//     slam::BAResidual r;
//
//     double q[4];
//     double c[3];
//     double x[3];
//     double e[1];
//
//     // setup
//     K << 279.0161682343449, 0, 150.3072895826164,
//          0, 276.3467561622266, 123.3623526538343,
//          0, 0, 1;
//     x1 << 130, 62;
//     x2 << 131.474, 73.2267;
//
//     q[0] = 0.0;
//     q[1] = 0.0;
//     q[2] = 0.0;
//     q[3] = 0.0;
//
//     c[0] = 0.0;
//     c[1] = 0.0;
//     c[2] = 0.0;
//
//     x[0] = 0.0;
//     x[1] = 0.0;
//     x[2] = 1.0;
//
//     r.K = K;
//     r.x1 = x1;
//     r.x2 = x2;
//     r(q, c, x, e);
//     std::cout << e[0] << std::endl;
// }

TEST(BundleAdjustment, constructor)
{
    slam::BundleAdjustment ba;
    ASSERT_EQ(false, ba.configured);
}

TEST(BundleAdjustment, configure)
{
    slam::Mat3 K;
    slam::MatX x1_pts, x2_pts;
    slam::BundleAdjustment ba;

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
    slam::BundleAdjustment ba;

    // setup
    slam::csv2mat(TEST_DATA_1, false, x1_pts);
    slam::csv2mat(TEST_DATA_2, false, x2_pts);
    K << 279.0161682343449, 0, 150.3072895826164,
         0, 276.3467561622266, 123.3623526538343,
         0, 0, 1;
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
