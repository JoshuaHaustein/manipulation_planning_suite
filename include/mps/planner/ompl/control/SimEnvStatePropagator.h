//
// Created by joshua on 8/21/17.
//

#ifndef MANIPULATION_PLANNING_SUITE_SIMENVSTATEPROPAGATOR_H
#define MANIPULATION_PLANNING_SUITE_SIMENVSTATEPROPAGATOR_H

// ompl includes
#include <ompl/control/StatePropagator.h>
#include <ompl/control/SpaceInformation.h>
// sim_env includes
#include <sim_env/SimEnv.h>
#include <sim_env/Controller.h>
// mps includes
#include <mps/planner/ompl/state/SimEnvState.h>

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
                    SimEnvStatePropagator(::ompl::control::SpaceInformationPtr si,
                                          sim_env::WorldPtr world,
                                          sim_env::RobotVelocityControllerPtr controller,
                                          bool semi_dynamic=true,
                                          float t_max=8.0f);
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
                    mutable sim_env::RobotVelocityControllerPtr _controller;
                    bool _semi_dynamic;
                    float _t_max;
                    state::SimEnvWorldStateSpaceConstWeakPtr _state_space;
                };
            }
        }
    }
}


#endif //MANIPULATION_PLANNING_SUITE_SIMENVSTATEPROPAGATOR_H
