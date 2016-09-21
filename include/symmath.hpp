#ifndef __SLAM_SYMMATH_HPP__
#define __SLAM_SYMMATH_HPP__

#include <iostream>
#include <vector>
#include <fstream>

#include <ginac/ginac.h>


class SymMath
{
public:
    static void outputMotionModelJacobian(
        std::string file_path,
        std::vector<GiNaC::ex> model,
        std::vector<GiNaC::symbol> states
    );
};

#endif
