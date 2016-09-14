#include "model.hpp"


TwoWheelRobot::TwoWheelRobot(void)
{
    this->initialized = false;
}


VecXd TwoWheelRobot::gFunc(VecXd x, VecXd u, float dt)
{
    VecXd g;

    g << x(1) + u(1) * cos(x(3)) * dt,
         x(2) + u(1) * sin(x(3)) * dt,
         x(3) + u(2) * dt;

    return g;
}

MatXd TwoWheelRobot::GFunc(VecXd x, VecXd u, float dt)
{
    MatXd G;

    G << 1.0, 0.0, (-u(1) * sin(x(3)) * dt),
         0.0, 1.0, (u(1) * cos(x(3)) * dt),
         0.0, 0.0, 1.0;

    return G;
}

VecXd TwoWheelRobot::hFunc(VecXd x)
{
    VecXd h;
    MatXd H;

    H = Eigen::MatrixXd::Identity(3, 3);
    h = H * x;

    return h;
}

MatXd TwoWheelRobot::HFunc(VecXd y)
{
    MatXd H;

    H = Eigen::MatrixXd::Identity(3, 3);

    return H;
}
