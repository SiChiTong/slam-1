#include "model.hpp"


TwoWheelRobot::TwoWheelRobot(void)
{
    this->initialized = false;
}

VecXf TwoWheelRobot::gFunc(VecXf x, VecXf u, float dt)
{
    VecXf g;

    g << x(1) + u(1) * cos(x(3)) * dt,
         x(2) + u(1) * sin(x(3)) * dt,
         x(3) + u(2) * dt;

    return g;
}

MatXf TwoWheelRobot::GFunc(VecXf x, VecXf u, float dt)
{
    MatXf G;

    G << 1.0, 0.0, (-u(1) * sin(x(3)) * dt),
         0.0, 1.0, (u(1) * cos(x(3)) * dt),
         0.0, 0.0, 1.0;

    return G;
}

VecXf TwoWheelRobot::hFunc(VecXf x)
{
    VecXf h;
    MatXf H;

    H = Eigen::MatrixXf::Identity(3, 3);
    h = H * x;

    return h;
}

MatXf TwoWheelRobot::HFunc(VecXf y)
{
    MatXf H;

    H = Eigen::MatrixXf::Identity(3, 3);

    return H;
}
