#ifndef __SLAM_ESTIMATION_HPP__
#define __SLAM_ESTIMATION_HPP__

#include <iostream>
#include <functional>

#include <Eigen/Geometry>


#ifndef __EIGEN_TYPEDEF__
#define __EIGEN_TYPEDEF__
  typedef Eigen::Vector2f Vec2f;
  typedef Eigen::Vector3f Vec3f;
  typedef Eigen::VectorXf VecXf;

  typedef Eigen::Matrix3f Mat3f;
  typedef Eigen::MatrixXf MatXf;
#endif



class KalmanFilter
{
public:
    bool initialized;
    VecXf mu;

    MatXf B;
    MatXf R;

    MatXf C;
    MatXf Q;

    MatXf S;
    MatXf I;
    MatXf K;

    VecXf mu_p;
    MatXf S_p;

    KalmanFilter(void);
    int init(
        VecXf mu,
        MatXf R,
        MatXf C,
        MatXf Q
    );
    int estimate(MatXf A, VecXf y);
};



class ExtendedKalmanFilter
{
public:
    bool initialized;
    VecXf mu;

    MatXf R;
    MatXf Q;

    MatXf S;
    MatXf I;
    MatXf K;

    VecXf mu_p;
    MatXf S_p;

    ExtendedKalmanFilter(void);
    int init(VecXf mu, MatXf R, MatXf Q);
    int predictionUpdate(VecXf g, MatXf G);
    int measurementUpdate(VecXf h, MatXf H, VecXf y);
};



class ParticleFilter
{
public:
    bool initialized;
    VecXf mu;

    int M;

    // VecXf mu_p;
    // MatXf S_p;

    ParticleFilter(void);
    int init(int M, VecXf mu);
    int estimate(
        std::vector<VecXf> X_p,
        std::vector<VecXf> hX_p,
        VecXf y
    );
};

#endif
