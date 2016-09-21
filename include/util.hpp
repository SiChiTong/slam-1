#ifndef __SLAM_UTIL_HPP__
#define __SLAM_UTIL_HPP__

#include <stdio.h>
#include <math.h>
#include <time.h>


// MACROS
#define UNUSED(expr) do { (void)(expr); } while (0)


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
