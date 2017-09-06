//
// Created by joshua on 9/6/17.
//
#include <mps/planner/util/Math.h>
#include <cmath>
#include <boost/math/constants/constants.hpp>

void mps::planner::util::math::normalize_orientation(double& value) {
    // from ompl::base::SO2StateSpace::enforceBounds
    double v = fmod(value, 2.0 * boost::math::constants::pi<double>());
    if (v < -boost::math::constants::pi<double>())
        v += 2.0 * boost::math::constants::pi<double>();
    else if (v >= boost::math::constants::pi<double>())
        v -= 2.0 * boost::math::constants::pi<double>();
    value = v;
}
