#include <gtest/gtest.h>

#include "slam/viz/viz.hpp"


slam::Viz viz;


TEST(Viz, constructor)
{
    ASSERT_EQ(false, viz.configured);
}

TEST(Viz, configure)
{
    viz.configure();
    ASSERT_EQ(true, viz.configured);
}

TEST(Viz, run)
{
    viz.run();
}


int main(int argc, char* argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
