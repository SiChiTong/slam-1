#ifndef __SLAM_UTILS_UTILS_HPP__
#define __SLAM_UTILS_UTILS_HPP__

#include <stdio.h>
#include <math.h>
#include <time.h>

#include <iostream>

#include "slam/utils/math.hpp"


namespace slam {

// MACROS
#define UNUSED(expr) do { (void)(expr); } while (0)


// FUNCTIONS
void rmtrailslash(std::string &path);

} // end of slam namespace
#endif
