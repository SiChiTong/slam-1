#include <gtest/gtest.h>

#include "slam/kinematics/quadrotor.hpp"
#include "slam/symmath/symmath.hpp"


TEST(SymbolicDifferentiation, test)
{
    GiNaC::symbol x("x");
    GiNaC::symbol y("y");
    GiNaC::ex P;

    P = pow(x, 5) + pow(x, 2) + y;

    // std::cout << P.diff(x) << std::endl;  // diff P wrt x

    slam::QuadrotorModel quad;

    // quad.generateMotionModelJacobian();
}

int main(int argc, char* argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
