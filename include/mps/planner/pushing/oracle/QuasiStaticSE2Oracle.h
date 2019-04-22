#pragma once

#include <mps/planner/pushing/oracle/Oracle.h>

namespace mps {
namespace planner {
    namespace pushing {
        namespace oracle {
            /**
             * Hand designed pushing oracle for holonomic 2D robots and objects in SE(2).
             * This oracle is based on Zhou and Mason's work 
             * "Pushing revisited: Differential flatness, trajectory planning and stabilization".
             * It provides a policy for stable pushing of polygonal objects.
             */
            class QuasiStaticSE2Oracle : public PushingOracle {
            public:
                struct Parameters {
                    Parameters(); // creates default values
                    float eps_min; // minimal half width of a pushing edge
                    float eps_dist; // distance between pusher and object from which on to consider both to be in contact
                };

                QuasiStaticSE2Oracle(const std::vector<sim_env::ObjectPtr>& objects, unsigned int robot_id);
                ~QuasiStaticSE2Oracle();

                /**
                     * Predicts what action should be taken in the current robot state
                     * in order to move the object from its current state to the desired target state.
                     * @param current_state - current state
                     * @param next_state - desired next state
                     * @param obj_id - object id identifying the object to be pushed
                     * @param control - output control that is directed towards next_state
                     */
                void predictAction(const mps::planner::ompl::state::SimEnvWorldState* current_state,
                    const mps::planner::ompl::state::SimEnvWorldState* target_state,
                    const unsigned int& obj_id, ::ompl::control::Control* control) override;

                /**
                     * Samples a robot state that is likely to be feasible for applying this oracle.
                     * In other words, this function samples from the feasibility distribution.
                     * @param current_obj_state - current object state
                     * @param next_obj_state - desired future object state
                     * @param obj_id - object id identifying the object to be pushed
                     * @param new_robot_state  - current robot state
                     */
                void samplePushingState(const mps::planner::ompl::state::SimEnvWorldState* current_state,
                    const mps::planner::ompl::state::SimEnvWorldState* next_state,
                    const unsigned int& obj_id,
                    mps::planner::ompl::state::SimEnvObjectState* new_robot_state) override;

            private:
                unsigned int _robot_id;
            };
        }
    }
}
}