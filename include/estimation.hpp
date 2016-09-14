#ifndef __SLAM_ESTIMATION_HPP__
#define __SLAM_ESTIMATION_HPP__

#include <iostream>
#include <functional>

#include <Eigen/Geometry>


#ifndef __EIGEN_TYPEDEF__
#define __EIGEN_TYPEDEF__
  typedef Eigen::Vector2d Vec2d;
  typedef Eigen::Vector3d Vec3d;
  typedef Eigen::VectorXd VecXd;

  typedef Eigen::Matrix3d Mat3d;
  typedef Eigen::MatrixXd MatXd;
#endif



class KalmanFilter
{
public:
    bool initialized;
    VecXd mu;

    MatXd B;
    MatXd R;

    MatXd C;
    MatXd Q;

    MatXd S;
    MatXd I;
    MatXd K;

    VecXd mu_p;
    MatXd S_p;

    KalmanFilter(void);
    int init(
        VecXd mu,
        MatXd R,
        MatXd C,
        MatXd Q
    );
    int estimate(MatXd A, VecXd y);
};



class ExtendedKalmanFilter
{
public:
    bool initialized;
    VecXd mu;

    MatXd R;
    MatXd Q;

    MatXd S;
    MatXd I;
    MatXd K;

    VecXd mu_p;
    MatXd S_p;


    ExtendedKalmanFilter(void);
    int init(VecXd mu, MatXd R, MatXd Q);
    int predictionUpdate(VecXd, g, MatXd G, VecXd u, float dt);
    int measurementUpdate(VecXd, h, MatXd H, VecXd y);
};



class ParticleFilter
{
public:
    bool initialized;
    VecXd mu;

    MatXd R;
    MatXd Q;

    int M;
    MatXd X_p;
    MatXd w;
    MatXd S;

    VecXd mu_p;
    MatXd S_p;


    ParticleFilter(void);
    int init(int M, VecXd mu, MatXd R, MatXd Q);
};

#endif
