#include "slam/utils/math.hpp"


namespace slam {

int sign(double x)
{
    return (x < 0) ? -1 : 1;
}

double C(double x)
{
    return cos(x);
}

double S(double x)
{
    return sin(x);
}

double T(double x)
{
    return tan(x);
}

float deg2rad(float d)
{
    return d * (M_PI / 180);
}

float rad2deg(float r)
{
    return r * (180 / M_PI);
}

int fltcmp(float f1, float f2)
{
    if (fabs(f1 - f2) <= 0.0001) {
        return 0;
    } else if (f1 > f2) {
        return 1;
    } else {
        return -1;
    }
}

MatX kronecker_product(MatX A, MatX B)
{
    MatX C;
    double a;

    // setup
    C.resize((A.rows() * B.rows()), (A.cols() * B.cols()));

    // calculate kronecker product
    for (int i = 0; i < A.rows(); i++) {
        for (int j = 0; j < A.cols(); j++) {
            a = A(i, j);
            C.block(i * B.rows(), j * B.cols(), B.rows(), B.cols()) = a * B;
        }
    }

    return C;
}

}  // end of slam namespace
