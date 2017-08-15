//
// Created by joshua on 8/14/17.
//

#ifndef MANIPULATION_PLANNING_SUITE_NAIVECONTROLSAMPLER_H
#define MANIPULATION_PLANNING_SUITE_NAIVECONTROLSAMPLER_H

#include <ompl/control/SimpleDirectedControlSampler.h>

namespace mps {
    namespace planner {
        namespace ompl {
            namespace control {
                class NaiveControlSampler {
                    // TODO this is is similiar to ompl's SimpleDirectedControlSampler,
                    // TODO i.e. sample k controls and return the best. Maybe we can use composition or
                    // TODO maybe this class is not required.
                    // TODO It has to operate on RealValueParameterizedControl though.
                };
            }
        }
    }
}
#endif //MANIPULATION_PLANNING_SUITE_NAIVECONTROLSAMPLER_H
