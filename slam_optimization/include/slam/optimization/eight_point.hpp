#ifndef __SLAM_OPTIMIZATION_EIGHT_POINT_HPP__
#define __SLAM_OPTIMIZATION_EIGHT_POINT_HPP__

#include <iostream>
#include <math.h>

#include <Eigen/Dense>

#include "slam/utils/utils.hpp"


namespace slam {
namespace optimization {

// FUNCTIONS
MatX kronecker_product(MatX A, MatX B);

// CLASSES
class EightPoint
{
public:
    bool configured;

    int image_width;
    int image_height;
    Mat3 N;

    EightPoint(void);
    int configure(int image_width, int image_height);
    void normalizePoints(MatX &pts1, MatX &pts2);
    void formMatrixA(MatX &pts1, MatX &pts2, MatX &A);
    void approximateFundamentalMatrix(MatX &A, MatX &F);
    void refineFundamentalMatrix(MatX &F);
    void denormalizeFundamentalMatrix(MatX &F);
    int estimate(MatX &pts1, MatX &pts2);
};

}  // end of optimization namespace
}  // end of slam namespace
#endif
