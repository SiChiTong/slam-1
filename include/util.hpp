#ifndef __SLAM_UTIL_HPP__
#define __SLAM_UTIL_HPP__

#include <stdio.h>
#include <math.h>
#include <time.h>

#include <Eigen/Geometry>


// MACROS
#define UNUSED(expr) do { (void)(expr); } while (0)


#ifndef __EIGEN_TYPEDEF__
#define __EIGEN_TYPEDEF__
  typedef Eigen::Vector2f Vec2;
  typedef Eigen::Vector3f Vec3;
  typedef Eigen::VectorXf VecX;

  typedef Eigen::Matrix2f Mat2;
  typedef Eigen::Matrix3f Mat3;
  typedef Eigen::MatrixXf MatX;
#endif


// FUNCTIONS
float deg2rad(float d);
float rad2deg(float r);
int fltcmp(float f1, float f2);
void tic(struct timespec *tic);
float toc(struct timespec *tic);
double C(double x);
double S(double x);
double T(double x);

#endif
