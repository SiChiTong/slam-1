#ifndef __SLAM_SYMMATH_SYMMATH_HPP__
#define __SLAM_SYMMATH_SYMMATH_HPP__

#include <iostream>
#include <vector>
#include <fstream>

#include <ginac/ginac.h>


namespace slam {

class SymMath
{
public:
    static void outputMotionModelJacobian(
        std::string file_path,
        std::vector<GiNaC::ex> model,
        std::vector<GiNaC::symbol> states
    );
};

} // end of slam namespace
#endif