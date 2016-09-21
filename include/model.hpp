#ifndef __SLAM_MODEL_HPP__
#define __SLAM_MODEL_HPP__

#include <Eigen/Geometry>
#include <ginac/ginac.h>

#include "util.hpp"
#include "symmath.hpp"


#ifndef __EIGEN_TYPEDEF__
#define __EIGEN_TYPEDEF__
  typedef Eigen::Vector2f Vec2f;
  typedef Eigen::Vector3f Vec3f;
  typedef Eigen::VectorXf VecXf;

  typedef Eigen::Matrix3f Mat3f;
  typedef Eigen::MatrixXf MatXf;
#endif


class TwoWheelRobotModel
{
public:
    bool initialized;

    TwoWheelRobotModel(void);
    VecXf gFunc(VecXf x, VecXf u, float dt);
    MatXf GFunc(VecXf x, VecXf u, float dt);
    VecXf hFunc(VecXf x);
    MatXf HFunc(VecXf y);
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
    VecXf gFunc(VecXf x, VecXf u, float dt);
    MatXf GFunc(VecXf x, VecXf u, float dt);
    VecXf hFunc(VecXf x);
    MatXf HFunc(VecXf y);
};

#endif
