#ifndef __SLAM_MODEL_HPP__
#define __SLAM_MODEL_HPP__

#include <Eigen/Geometry>


#ifndef __EIGEN_TYPEDEF__
#define __EIGEN_TYPEDEF__
  typedef Eigen::Vector2d Vec2d;
  typedef Eigen::Vector3d Vec3d;
  typedef Eigen::VectorXd VecXd;

  typedef Eigen::Matrix3d Mat3d;
  typedef Eigen::MatrixXd MatXd;
#endif


class TwoWheelRobot
{
public:
    bool initialized;

    TwoWheelRobot(void);
    VecXd gFunc(VecXd x, VecXd u, float dt);
    MatXd GFunc(VecXd x, VecXd u, float dt);
    VecXd hFunc(VecXd x);
    MatXd HFunc(VecXd y);
};

#endif
