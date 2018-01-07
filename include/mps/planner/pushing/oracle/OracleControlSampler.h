//
// Created by joshua on 8/14/17.
//

#ifndef MANIPULATION_PLANNING_SUITE_ORACLECONTROLSAMPLER_H
#define MANIPULATION_PLANNING_SUITE_ORACLECONTROLSAMPLER_H

#include <ompl/control/SpaceInformation.h>
#include <mps/planner/ompl/control/Interfaces.h>
#include <mps/planner/pushing/oracle/Oracle.h>
#include <mps/planner/ompl/state/SimEnvState.h>
#include <mps/planner/util/Random.h>

namespace mps {
    namespace planner {
        namespace pushing {
            namespace oracle {
                // The oracle control sampler utilizes an oracle to sample a sequence of controls for a
                // pushing rearrangement problem.
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
                     * Samples a series of controls moving the system from the source state towards the destination state.
                     * The sampler utilizes an oracle and a robot steering function to do this.
                     * The decision on what to do is based on local_target_obj, which is the index
                     * denoting the sim_env object that is most relevant to be moved towards its state in dest.
                     *
                     * @param controls - an empty vector, which after the execution contains one or more controls
                     *                  The controls are read only and will be reused when this function is called again
                     * @param source - source state
                     * @param dest - destination state
                     * @param local_target_obj - id identifying on which object the steering should focus
                     */
                    void sampleTo(std::vector<::ompl::control::Control const*>& controls,
                                  const ::ompl::base::State* source,
                                  const ::ompl::base::State* dest,
                                  unsigned int local_target_obj);
                    /**
                     * Samples a robot state from oracle feasibility p(x_r | x_t, x'_t)
                     * @param sample_state - contains x_t and as a result will contain the sampled x_r. WARNING: Must be an instance of SimEnvState
                     * @param target_state - contains x'_t. WARNING: Must be an instance of SimEnvState
                     * @param local_target_obj - object identifier t
                     * @param p_uniform - if > 0.0f, feasible state is sampled uniformly rather than using oracle with
                     *          probability p_uniform;.
                     */
                    void sampleFeasibleState(::ompl::base::State* x_state,
                                              const ::ompl::base::State* x_prime_state,
                                              unsigned int local_target_obj,
                                              const float& p_uniform=0.0f);
                    float getFeasibility(const ::ompl::base::State* x_state,
                                         const ::ompl::base::State* x_prime_state,
                                         unsigned int active_obj_id) const;

                    /**
                     * Steers the robot to a given goal state.
                     */
                    bool steerRobot(std::vector<::ompl::control::Control const*>& controls,
                                    const ::ompl::base::State* source,
                                    const ::ompl::base::State* dest);
                    bool steerRobot(std::vector<::ompl::control::Control const*>& controls,
                                    const mps::planner::ompl::state::SimEnvWorldState* source,
                                    const mps::planner::ompl::state::SimEnvWorldState* dest);
                    bool steerRobot(std::vector<::ompl::control::Control const*>& controls,
                                    const mps::planner::ompl::state::SimEnvWorldState* source,
                                    const mps::planner::ompl::state::SimEnvObjectState* dest);
                    /**
                     * Steers the system to the given state by trying to reduce the distance
                     * for object obj_id. This method essentially just calls the pushing oracle
                     * and asks for a pushing action.
                     * @param controls - will contain controls provided by oracle
                     * @param source - state from which to execute push from
                     * @param dest - state towards which to execute push to
                     * @param obj_id - id of the object to push
                     * @param action_noise - probability to sample an action uniformly at random rather than using the oracle
                     */
                    bool steerPush(std::vector<::ompl::control::Control const*>& controls,
                                   const ::ompl::base::State* source,
                                   const ::ompl::base::State* dest,
                                   unsigned int obj_id,
                                   const float& action_noise=0.0f);
                    bool steerPush(std::vector<::ompl::control::Control const*>& controls,
                                   const mps::planner::ompl::state::SimEnvWorldState* source,
                                   const mps::planner::ompl::state::SimEnvWorldState* dest,
                                   unsigned int obj_id,
                                   const float& action_noise=0.0f);
                    // bool steerPushSimple(std::vector<::ompl::control::Control const*>& controls,
                    //                      const ::ompl::base::State* source,
                    //                      const ::ompl::base::State* dest,
                    //                      unsigned int obj_id);
                    // bool steerPushSimple(std::vector<::ompl::control::Control const*>& controls,
                    //                const mps::planner::ompl::state::SimEnvWorldState* source,
                    //                const mps::planner::ompl::state::SimEnvWorldState* dest,
                    //                unsigned int obj_id);
                    void randomControl(std::vector<::ompl::control::Control const*>& controls);

                private:
                    ::ompl::control::SpaceInformationPtr _si;
                    ::ompl::control::ControlSamplerPtr _control_sampler;
                    mps::planner::pushing::oracle::RobotOraclePtr _robot_oracle;
                    mps::planner::pushing::oracle::PushingOraclePtr _pushing_oracle;
                    mps::planner::ompl::state::SimEnvObjectState* _robot_state;
                    mps::planner::ompl::state::SimEnvObjectStateSpacePtr _robot_state_space;
                    ::ompl::base::StateSamplerPtr _robot_state_sampler;
                    std::size_t _control_idx;
                    std::vector<mps::planner::ompl::control::RealValueParameterizedControl*> _controls;
                    unsigned int _robot_id;
                    ::ompl::RNGPtr _rng;

                    inline mps::planner::ompl::control::RealValueParameterizedControl* getControl();
                    inline void resetControlIdx();
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
