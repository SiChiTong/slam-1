#ifndef __SLAM_UTIL_HPP__
#define __SLAM_UTIL_HPP__

#include <stdio.h>
#include <math.h>
#include <time.h>


// FUNCTIONS
float deg2rad(float d);
float rad2deg(float r);
int dblcmp(float f1, float f2);
void tic(struct timespec *tic);
float toc(struct timespec *tic);

#endif
