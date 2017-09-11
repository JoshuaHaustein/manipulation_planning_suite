//
// Created by joshua on 9/6/17.
//

#ifndef MANIPULATION_PLANNING_SUITE_MATH_H
#define MANIPULATION_PLANNING_SUITE_MATH_H

namespace mps {
    namespace planner {
        namespace util {
            namespace math {
                void normalize_orientation(double& value);
                float shortest_direction_so2(float val_1, float val_2);
            }
        }
    }
}
#endif //MANIPULATION_PLANNING_SUITE_MATH_H
