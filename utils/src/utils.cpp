#include "slam/utils/utils.hpp"


namespace slam {

float deg2rad(float d)
{
    return d * (M_PI / 180);
}

float rad2deg(float r)
{
    return r * (180 / M_PI);
}

int dblcmp(float f1, float f2)
{
    if (fabs(f1 - f2) <= 0.0001) {
        return 0;
    } else if (f1 > f2) {
        return 1;
    } else {
        return -1;
    }
}

void tic(struct timespec *tic)
{
    clock_gettime(CLOCK_MONOTONIC, tic);
}

float toc(struct timespec *tic)
{
    struct timespec toc;
    float time_elasped;

    clock_gettime(CLOCK_MONOTONIC, &toc);
    time_elasped = (toc.tv_sec - tic->tv_sec);
    time_elasped += (toc.tv_nsec - tic->tv_nsec) / 1000000000.0;

    return time_elasped;
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

void rmtrailslash(std::string &path)
{
    if (path.length() > 0) {
        std::string::iterator it = path.end() - 1;

        if (*it == '/') {
            path.erase(it);
        }
    }
}

} // end of slam namespace
