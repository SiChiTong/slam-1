#ifndef __SLAM_OPTIMIZATION_LLS_HPP__
#define __SLAM_OPTIMIZATION_LLS_HPP__


namespace slam {

class LLSSolver
{
public:
    bool configured;

    LLSSolver(void);
    int configure(void);
    int solve(void);
};

}  // end of slam namespace
#endif
