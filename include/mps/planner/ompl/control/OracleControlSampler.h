//
// Created by joshua on 8/14/17.
//

#ifndef MANIPULATION_PLANNING_SUITE_ORACLECONTROLSAMPLER_H
#define MANIPULATION_PLANNING_SUITE_ORACLECONTROLSAMPLER_H

#include <ompl/control/DirectedControlSampler.h>
#include <mps/planner/ompl/control/Interfaces.h>
#include <mps/planner/oracle/Oracle.h>

namespace mps {
    namespace planner {
        namespace ompl {
            namespace control {
                class OracleControlSampler : public ::ompl::control::DirectedControlSampler {
                    // TODO implement oracle control sampler here; use oracle to sample a control
                    // TODO operate on RealValueParameterizedControl
                };
            }
        }
    }
}
#endif //MANIPULATION_PLANNING_SUITE_ORACLECONTROLSAMPLER_H
