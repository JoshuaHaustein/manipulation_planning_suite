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

/**
 * Returns the shortest direction from val_1 to val_2 assuming that both values
 * are angles in radian in [-pi, pi)
 * @param val_1
 * @param val_2
 * @return the smallest change in angle da such that val_1 + da = val_2 mod 2pi
 */
float mps::planner::util::math::shortest_direction_so2(float val_1, float val_2) {
    float value = val_2 - val_1;
    if (std::abs(value) > boost::math::constants::pi<float>()) {
        if (value > 0.0f) { // val_2 > val_1
            value -= 2.0f * boost::math::constants::pi<float>();
        } else {
            value += 2.0f * boost::math::constants::pi<float>();
        }
    }
    return value;
}
