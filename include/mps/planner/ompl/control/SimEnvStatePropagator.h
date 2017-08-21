//
// Created by joshua on 8/21/17.
//

#ifndef MANIPULATION_PLANNING_SUITE_SIMENVSTATEPROPAGATOR_H
#define MANIPULATION_PLANNING_SUITE_SIMENVSTATEPROPAGATOR_H

#include <ompl/control/StatePropagator.h>
#include <ompl/control/SpaceInformation.h>
#include <sim_env/SimEnv.h>

namespace mps {
    namespace planner {
        namespace ompl {
            namespace control {
                // TODO implement state propagator using sim_env. The state propagator operates on VelocityControls
                /**
                 * State propagator that operates on a sim_env world.
                 * It only supports SimEnvWorldState as state type and VelocityControl as control types.
                 */
                class SimEnvStatePropagator : public ::ompl::control::StatePropagator {
                public:
                    SimEnvStatePropagator(::ompl::control::SpaceInformationPtr si, sim_env::WorldPtr world);
                    ~SimEnvStatePropagator();

                    // default ompl propagate function
                    void propagate(const ::ompl::base::State* state, const ::ompl::control::Control *control,
                                   double duration, ::ompl::base::State* result) const override;

                    /**
                     * Semi-dynamic propagate function that adapts the provided control to include
                     * optional time_to_rest times.
                     * @param state - start state (must be of type SimEnvWorldState*)
                     * @param control - control to apply (must be of type VelocityControl)
                     * @param result - the resulting state (must not be null)
                     */
                    bool propagate(const ::ompl::base::State* state, ::ompl::control::Control *control,
                                   ::ompl::base::State* result) const;

                    bool canPropagateBackward() const override;

                private:
                    mutable sim_env::WorldPtr _world;
                };
            }
        }
    }
}


#endif //MANIPULATION_PLANNING_SUITE_SIMENVSTATEPROPAGATOR_H
