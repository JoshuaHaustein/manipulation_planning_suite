//
// Created by joshua on 9/6/17.
//
#include <boost/math/constants/constants.hpp>
#include <cmath>
#include <mps/planner/util/Math.h>

double mps::planner::util::math::normalize_orientation(double v)
{
    // from ompl::base::SO2StateSpace::enforceBounds
    v = fmod(v, 2.0 * boost::math::constants::pi<double>());
    if (v < -boost::math::constants::pi<double>())
        v += 2.0 * boost::math::constants::pi<double>();
    else if (v >= boost::math::constants::pi<double>())
        v -= 2.0 * boost::math::constants::pi<double>();
    return v;
}

float mps::planner::util::math::normalize_orientation(float v)
{
    v = fmod(v, 2.0f * boost::math::constants::pi<float>());
    if (v < -boost::math::constants::pi<float>())
        v += 2.0f * boost::math::constants::pi<float>();
    else if (v >= boost::math::constants::pi<float>())
        v -= 2.0f * boost::math::constants::pi<float>();
    return v;
}

float mps::planner::util::math::shortest_direction_so2(float val_1, float val_2)
{
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

Eigen::Vector2f mps::planner::util::math::normal(const Eigen::Vector2f& vec)
{
    static float epsilon = 1e-5f;
    Eigen::Vector2f normal(0.0f, 0.0f);
    if (std::abs(vec[0]) <= epsilon and std::abs(vec[1]) <= epsilon) {
        return normal;
    } else if (std::abs(vec[0]) <= epsilon) {
        normal[0] = 1.0f;
        normal[1] = -vec[0] / vec[1];
        normal.normalize();
        return normal;
    }
    normal[0] = -vec[1] / vec[0];
    normal[1] = 1.0f;
    normal.normalize();
    return normal;
}

float mps::planner::util::math::projectToSO2Range(float lower, float upper, float v)
{
    if (upper < lower) {
        if (v > upper and v < lower) {
            v = v - upper < lower - v ? upper : lower;
        }
    } else if (v < lower or v > upper) {
        float dlower = std::abs(shortest_direction_so2(v, lower));
        float dupper = std::abs(shortest_direction_so2(v, upper));
        v = dlower < dupper ? dlower : dupper;
    }
    return v;
}
