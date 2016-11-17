#include <functional>

#include <gtest/gtest.h>

#include "slam/optimization/ba.hpp"


TEST(BAResidual, calculate)
{

}


TEST(BundleAdjustment, constructor)
{
    slam::BundleAdjustment ba;
    ASSERT_EQ(false, ba.configured);
}

TEST(BundleAdjustment, configure)
{
    slam::BundleAdjustment ba;

    // ba.configure();
    // ASSERT_EQ(true, ba.configured);
}

// TEST(BundleAdjustment, solve)
// {
//     slam::BundleAdjustment ba;
//
//     ba.configure();
//     ba.solve();
// }

int main(int argc, char* argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
