#include "slam/optimization/ba.hpp"


namespace slam {

BundleAdjustment::BundleAdjustment(void)
{
    this->configured = false;

}

int BundleAdjustment::configure(void)
{
    this->configured = true;

    return 0;
}

}  // end of slam namespace
