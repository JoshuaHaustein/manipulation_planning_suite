//
// Created by joshua on 8/14/17.
//

#ifndef MANIPULATION_PLANNING_SUITE_ORACLECONTROLSAMPLER_H
#define MANIPULATION_PLANNING_SUITE_ORACLECONTROLSAMPLER_H

#include <ompl/control/SpaceInformation.h>
#include <mps/planner/ompl/control/Interfaces.h>
#include <mps/planner/pushing/oracle/Oracle.h>
#include <mps/planner/ompl/state/SimEnvState.h>

namespace mps {
    namespace planner {
        namespace ompl {
            namespace control {
                class OracleControlSampler { //: public ::ompl::control::DirectedControlSampler {
                    // TODO implement oracle control sampler here; use oracle to sample a control
                    // TODO operate on RealValueParameterizedControl
                public:
                    struct Parameters {
                        float min_pushability;
                        float min_feasibility;
                        Parameters();
                    };

                    OracleControlSampler(::ompl::control::SpaceInformationPtr si,
                                        mps::planner::pushing::oracle::PushingOraclePtr oracle,
                                        mps::planner::pushing::oracle::RobotOraclePtr robot_oracle,
                                        const std::string& robot_name,
                                        const Parameters& params);
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
                                  ::ompl::base::State* dest,
                                  unsigned int local_target_obj);

                    // TODO make this protected?
                    void steerRobot(std::vector<::ompl::control::Control const*>& controls,
                                    const mps::planner::ompl::state::SimEnvWorldState* source,
                                    mps::planner::ompl::state::SimEnvWorldState* dest);

                    // TODO make this protected?
                    void steerPush(std::vector<::ompl::control::Control const*>& controls,
                                   const mps::planner::ompl::state::SimEnvWorldState* source,
                                   mps::planner::ompl::state::SimEnvWorldState* dest,
                                   unsigned int obj_id);

                    void randomAction(std::vector<::ompl::control::Control const*>& controls,
                                   const mps::planner::ompl::state::SimEnvWorldState* source,
                                   mps::planner::ompl::state::SimEnvWorldState* dest,
                                   unsigned int obj_id);
                    /**
                     * TODO not implemented
                     * TODO we should probably implement this rather than the custom method
                     * TODO this would require the active object information to be part of the state
                     * @param control
                     * @param source
                     * @param dest
                     * @return
                     */
//                    unsigned int sampleTo(::ompl::control::Control* control,
//                                          const ::ompl::base::State* source,
//                                          ::ompl::base::State* dest) override;
                    /**
                     * TODO not implemented
                     * @param control
                     * @param prev
                     * @param source
                     * @param dest
                     * @return
                     */
//                    unsigned int sampleTo(::ompl::control::Control* control,
//                                          const ::ompl::control::Control* prev,
//                                          const ::ompl::base::State* source,
//                                          ::ompl::base::State* dest) override;

                private:
                    ::ompl::control::SpaceInformationPtr _si;
                    mps::planner::pushing::oracle::RobotOraclePtr _robot_oracle;
                    mps::planner::pushing::oracle::PushingOraclePtr _pushing_oracle;
                    std::size_t _control_idx;
                    std::vector<mps::planner::ompl::control::RealValueParameterizedControl*> _controls;
                    unsigned int _robot_id;
                    Parameters _params;

                    inline mps::planner::ompl::control::RealValueParameterizedControl* getControl();
                    inline void resetControlIdx();
                };
            }
        }
    }
}
#endif //MANIPULATION_PLANNING_SUITE_ORACLECONTROLSAMPLER_H
