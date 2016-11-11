#include <functional>

#include <gtest/gtest.h>

#include "slam/optimization/gd.hpp"


slam::VecX beale(slam::VecX x)
{
    slam::VecX y(1);

    y << pow((1.5 - x(0) + x(0) * x(1)), 2)
         + pow((2.25 - x(0) + x(0) * pow(x(1), 2)), 2)
         + pow((2.625 - x(0) + x(0) * pow(x(1), 3)), 2);

    return y;
}

TEST(GDSolver, constructor)
{
    slam::GDSolver solver;
    ASSERT_EQ(solver.configured, false);
}

TEST(GDSolver, configure)
{
    int max_iter;
    slam::VecX eta(2);
    slam::VecX x(2);
    slam::GDSolver solver;

    max_iter = 1000;
    eta << 1.0, 1.0;
    x << 0.0, 0.0;

    solver.configure(max_iter, eta, x);
    ASSERT_EQ(solver.configured, true);
    ASSERT_EQ(solver.max_iter, 1000);
    ASSERT_FLOAT_EQ(solver.eta(0), 1.0);
}

TEST(GDSolver, solve)
{
    int max_iter;
    slam::VecX eta(2);
    slam::VecX x(2);
    slam::GDSolver solver;

    max_iter = 100000;
    eta << 0.006, 0.006;
    x << 0.0, 0.0;

    solver.configure(max_iter, eta, x);
    solver.diff_func = std::bind(diff_func, std::placeholders::_1);
    solver.f = std::bind(beale, std::placeholders::_1);
    solver.nb_functions = 1;
    solver.nb_unknowns = 2;
    solver.solve();

    ASSERT_TRUE(solver.x(0) > 2.8);
    ASSERT_TRUE(solver.x(0) <= 3.0);
    ASSERT_TRUE(solver.x(1) > 0.4);
    ASSERT_TRUE(solver.x(1) <= 0.5);
}

int main(int argc, char* argv[])
{
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
