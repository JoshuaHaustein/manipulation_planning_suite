//
// Created by joshua on 9/6/17.
//

#ifndef MANIPULATION_PLANNING_SUITE_MATH_H
#define MANIPULATION_PLANNING_SUITE_MATH_H

#include <Eigen/Core>
#include <algorithm>

namespace mps {
namespace planner {
    namespace util {
        namespace math {
            double normalize_orientation(double value);
            float normalize_orientation(float value);
            /**
             * Returns the shortest direction from val_1 to val_2 assuming that both values
             * are angles in radian in [-pi, pi)
             * @param val_1
             * @param val_2
             * @return the smallest change in angle da such that val_1 + da = val_2 mod 2pi
             */
            float shortest_direction_so2(float val_1, float val_2);
            template <typename T>
            void clampInplace(T& val, const T& min_val, const T& max_val)
            {
                val = std::max(std::min(val, max_val), min_val);
            }
            template <typename T>
            T clamp(const T& val, const T& min_val, const T& max_val)
            {
                return std::max(std::min(val, max_val), min_val);
            }

            Eigen::Vector2f normal(const Eigen::Vector2f& vec);
            /**
             * Project v into the range [lower, upper] in SO(2) with lower and upper in [-pi, pi).
             * The value lower may actually be larger than upper, i.e. 
             * the interval may range over the end of [-pi, pi). The interval [lower, upper] should 
             * be understood as minimal rotation, maximal rotation.
             */
            float projectToSO2Range(float lower, float upper, float v);
        }
    }
}
}
#endif //MANIPULATION_PLANNING_SUITE_MATH_H
