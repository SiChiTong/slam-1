#ifndef __SLAM_VIZ_VIZCAMERA_HPP__
#define __SLAM_VIZ_VIZCAMERA_HPP__

#include "slam/utils/utils.hpp"


namespace slam {

class VizCamera
{
public:
    bool configured;
    Vec3 position;
    Vec3 view;
    Vec3 up;

    VizCamera(void);
    int configure(void);
};

}  // end of slam namespace
#endif
