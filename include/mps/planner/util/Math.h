//
// Created by joshua on 9/6/17.
//

#ifndef MANIPULATION_PLANNING_SUITE_MATH_H
#define MANIPULATION_PLANNING_SUITE_MATH_H

#include <algorithm>

namespace mps {
    namespace planner {
        namespace util {
            namespace math {
                void normalize_orientation(double& value);
                void normalize_orientation(float& value);
                float shortest_direction_so2(float val_1, float val_2);
                template<typename T>
                void clampInplace(T& val, const T& min_val, const T& max_val) {
                    val = std::max(std::min(val, max_val), min_val);
                }
                template<typename T>
                T clamp(const T& val, const T& min_val, const T& max_val) {
                    return std::max(std::min(val, max_val), min_val);
                }
            }
        }
    }
}
#endif //MANIPULATION_PLANNING_SUITE_MATH_H
