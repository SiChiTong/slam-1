#ifndef __SLAM_MODEL_HPP__
#define __SLAM_MODEL_HPP__

#include <ginac/ginac.h>

#include "util.hpp"
#include "symmath.hpp"


class TwoWheelRobotModel
{
public:
    bool initialized;

    TwoWheelRobotModel(void);
    VecX gFunc(VecX x, VecX u, float dt);
    MatX GFunc(VecX x, VecX u, float dt);
    VecX hFunc(VecX x);
    MatX HFunc(VecX y);
};

class QuadrotorModel
{
public:
    bool initialized;

    float Ix;
    float Iy;
    float Iz;

    float ktau;
    float kt;

    float tauf;
    float taup;
    float tauq;
    float taur;

    float m;
    float g;

    QuadrotorModel(void);
    void generateMotionModelJacobian(void);
    VecX gFunc(VecX x, VecX u, float dt);
    MatX GFunc(VecX x, VecX u, float dt);
    VecX hFunc(VecX x);
    MatX HFunc(VecX y);
};

#endif
