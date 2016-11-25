#ifndef __SLAM_VIZ_VIZ_HPP__
#define __SLAM_VIZ_VIZ_HPP__

#include <GLFW/glfw3.h>

#include "slam/utils/utils.hpp"


namespace slam {

class VizSettings
{
public:
    int window_width;
    int window_height;
    std::string window_title;

    VizSettings(void);
};

class Viz
{
public:
    bool configured;

    VizSettings settings;

    Viz(void);
    int configure(void);
    int run(void);
};

}  // end of slam namespace
#endif
