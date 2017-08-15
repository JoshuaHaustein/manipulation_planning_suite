//
// Created by joshua on 8/14/17.
//

#ifndef MANIPULATION_PLANNING_SUITE_SIMENVSTATE_H
#define MANIPULATION_PLANNING_SUITE_SIMENVSTATE_H

namespace mps {
    namespace planner {
        namespace ompl {
            namespace state {
                class SimEnvStatePropagator {
                    // TODO implement state propagator using sim_env. The state propagator operates on VelocityControls
                };

                class SimEnvObjectStateSpace {
                    // TODO state space for sim env object based on active dofs
                };

                class SimEnvWorldStateSpace {
                    // TODO Compound state space representing world state
                };

                class SimEnvValidityChecker {
                    // TODO State validity checker using SimEnv
                };
            }
        }
    }
}
#endif //MANIPULATION_PLANNING_SUITE_SIMENVSTATE_H
