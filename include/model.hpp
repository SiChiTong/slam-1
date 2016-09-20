#ifndef __SLAM_MODEL_HPP__
#define __SLAM_MODEL_HPP__

#include <Eigen/Geometry>


#ifndef __EIGEN_TYPEDEF__
#define __EIGEN_TYPEDEF__
  typedef Eigen::Vector2f Vec2f;
  typedef Eigen::Vector3f Vec3f;
  typedef Eigen::VectorXf VecXf;

  typedef Eigen::Matrix3f Mat3f;
  typedef Eigen::MatrixXf MatXf;
#endif


class TwoWheelRobot
{
public:
    bool initialized;

    TwoWheelRobot(void);
    VecXf gFunc(VecXf x, VecXf u, float dt);
    MatXf GFunc(VecXf x, VecXf u, float dt);
    VecXf hFunc(VecXf x);
    MatXf HFunc(VecXf y);
};

#endif
