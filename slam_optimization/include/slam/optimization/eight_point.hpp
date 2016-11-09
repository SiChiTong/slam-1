#ifndef __SLAM_OPTIMIZATION_EIGHT_POINT_HPP__
#define __SLAM_OPTIMIZATION_EIGHT_POINT_HPP__


#include <iostream>
#include <vector>

#include <math.h>

#include <Eigen/Dense>

#include "slam/utils/utils.hpp"


namespace slam {
namespace optimization {

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
    int estimate(MatX pts1, MatX pts2, MatX &F);
    int estimate(MatX pts1, MatX pts2, Mat3 &K, Mat3 &E);
    int obtainPossiblePoses(Mat3 E, std::vector<MatX> &poses);
    int obtainPose(
        Vec3 pt1,
        Vec3 pt2,
        Mat3 K1,
        Mat3 K2,
        std::vector<MatX> poses,
        MatX &pose
    );
};

}  // end of optimization namespace
}  // end of slam namespace
#endif
