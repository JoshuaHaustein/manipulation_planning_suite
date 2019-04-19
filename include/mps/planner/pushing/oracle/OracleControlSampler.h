//
// Created by joshua on 8/14/17.
//

#ifndef MANIPULATION_PLANNING_SUITE_ORACLECONTROLSAMPLER_H
#define MANIPULATION_PLANNING_SUITE_ORACLECONTROLSAMPLER_H

#include <mps/planner/ompl/control/Interfaces.h>
#include <mps/planner/ompl/state/SimEnvState.h>
#include <mps/planner/pushing/oracle/Oracle.h>
#include <mps/planner/util/Random.h>
#include <ompl/control/SpaceInformation.h>

namespace mps {
namespace planner {
    namespace pushing {
        namespace oracle {
            // An OracleControlSampler provides access to a pushing and robot oracle.
            // All control arguments in this class need to be of the type provided by the SpaceInformation the object
            // is created with.
            class OracleControlSampler {
            public:
                OracleControlSampler(::ompl::control::SpaceInformationPtr si,
                    mps::planner::pushing::oracle::PushingOraclePtr oracle,
                    mps::planner::pushing::oracle::RobotOraclePtr robot_oracle,
                    const std::string& robot_name);
                OracleControlSampler(const OracleControlSampler& other) = delete;
                OracleControlSampler& operator=(const OracleControlSampler& other) = delete;
                ~OracleControlSampler();

                /**
                     * Sample a robot state to push object tid to its state in x_prime_state, given that 
                     * the world is in state x_state. The sampled robot state will be stored in x_state.
                     * @param sample_state - contains x_t and as a result will contain the sampled x_r. WARNING: Must be an instance of SimEnvState
                     * @param target_state - contains x'_t. WARNING: Must be an instance of SimEnvState
                     * @param tid - object identifier 
                     * @param p_uniform - if > 0.0f, pushing state is sampled uniformly rather than using oracle with
                     *          probability p_uniform;.
                     */
                void samplePushingState(ompl::state::SimEnvWorldState* x_state,
                    const ompl::state::SimEnvWorldState* x_prime_state,
                    unsigned int tid,
                    const float& p_uniform = 0.0f);
                // Convenience function of the function above. The states will be downcasted to SimEnvWorldState*
                void samplePushingState(::ompl::base::State* x_state,
                    const ::ompl::base::State* x_prime_state,
                    unsigned int tid,
                    const float& p_uniform = 0.0f);
                /**
                 *  Query the pushing policy, given that we are in state source and intend to push object
                 *  obj_id to its state in dest. The underlying pushing oracle determines the behaviour of this
                 * function. It may, or may not, consider obstacles when computing the pushing action.
                 * @param control - will contain the control suggested by pushing policy 
                 * @param source - SimEnvWorldState* to query policy for
                 * @param dest - SimEnvWorldState* extract target state for the object from this world state
                 */
                void queryPolicy(::ompl::control::Control* control,
                    const ompl::state::SimEnvWorldState* source,
                    const ompl::state::SimEnvWorldState* dest,
                    unsigned int obj_id,
                    float p_uniform = 0.0f);
                // Convenience function of the function above. The states will be downcasted to SimEnvWorldState*
                void queryPolicy(::ompl::control::Control* control,
                    const ::ompl::base::State* source,
                    const ::ompl::base::State* dest,
                    unsigned int obj_id,
                    float p_uniform = 0.0f);

                /**
                  * Compute a sequence of controls that move the robot from its state in source to its state in dest.
                  * The underlying robot_oracle determines whether this function considers obstacles, or not.
                  */
                void steerRobot(std::vector<::ompl::control::Control*>& controls,
                    const mps::planner::ompl::state::SimEnvWorldState* source,
                    const mps::planner::ompl::state::SimEnvWorldState* dest);
                // Convenience function of the function above. The states will be downcasted to SimEnvWorldState*
                void steerRobot(std::vector<::ompl::control::Control*>& controls,
                    const ::ompl::base::State* source,
                    const ::ompl::base::State* dest);
                void steerRobot(std::vector<::ompl::control::Control*>& controls,
                    const mps::planner::ompl::state::SimEnvWorldState* source,
                    const mps::planner::ompl::state::SimEnvObjectState* dest);
                /**
                 * Sample a random control and store it in controls.
                 */
                void randomControl(std::vector<::ompl::control::Control*>& controls);
                /**
                 * Sample a random control.
                 */
                void randomControl(::ompl::control::Control* control);

                // /**
                //      * Steers the system to the given state by trying to reduce the distance
                //      * for object obj_id. This method calls queryPolicy subsequently to obtain
                //      * a sequence of pushing actions.
                //      * @param controls - will contain controls provided by oracle
                //      * @param source - state from which to execute push from
                //      * @param dest - state towards which to execute push to
                //      * @param obj_id - id of the object to push
                //      * @param action_noise - probability to sample an action uniformly at random rather than using the oracle
                //      */
                // bool steerPush(std::vector<::ompl::control::Control const*>& controls,
                //     const ::ompl::base::State* source,
                //     const ::ompl::base::State* dest,
                //     unsigned int obj_id,
                //     const float& action_noise = 0.0f);
                // bool steerPush(std::vector<::ompl::control::Control const*>& controls,
                //     const mps::planner::ompl::state::SimEnvWorldState* source,
                //     const mps::planner::ompl::state::SimEnvWorldState* dest,
                //     unsigned int obj_id,
                //     const float& action_noise = 0.0f);

            private:
                ::ompl::control::SpaceInformationPtr _si;
                ::ompl::control::ControlSamplerPtr _control_sampler;
                mps::planner::pushing::oracle::RobotOraclePtr _robot_oracle;
                mps::planner::pushing::oracle::PushingOraclePtr _pushing_oracle;
                mps::planner::ompl::state::SimEnvObjectStateSpacePtr _robot_state_space;
                ::ompl::base::StateSamplerPtr _robot_state_sampler;
                unsigned int _robot_id;
                ::ompl::RNGPtr _rng;
            };
            typedef std::shared_ptr<OracleControlSampler> OracleControlSamplerPtr;
            typedef std::shared_ptr<const OracleControlSampler> OracleControlSamplerConstPtr;
            typedef std::weak_ptr<OracleControlSampler> OracleControlSamplerWeakPtr;
            typedef std::weak_ptr<const OracleControlSampler> OracleControlSamplerWeakConstPtr;
        }
    }
}
}
#endif //MANIPULATION_PLANNING_SUITE_ORACLECONTROLSAMPLER_H
