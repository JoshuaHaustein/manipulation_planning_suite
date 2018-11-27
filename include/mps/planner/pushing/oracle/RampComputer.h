//
// Created by joshua on 9/8/17.
//

#ifndef MANIPULATION_PLANNING_SUITE_RAMPCOMPUTER_H
#define MANIPULATION_PLANNING_SUITE_RAMPCOMPUTER_H

#include <mps/planner/pushing/oracle/Oracle.h>
#include <mps/planner/ompl/control/RampVelocityControl.h>
#include <mps/planner/ompl/state/SimEnvState.h>
#include <memory>

namespace mps {
    namespace planner {
            namespace pushing {
                namespace oracle {
                    class RampComputer : public RobotOracle {
                    public:
                        RampComputer(ompl::state::SimEnvObjectConfigurationSpacePtr robot_space,
                                     ompl::control::RampVelocityControlSpacePtr control_space);
                        RampComputer(const RampComputer& other);
                        ~RampComputer() override;
                        RampComputer& operator=(const RampComputer& other);

                        void steer(const Eigen::VectorXf &current_robot_state,
                                   const Eigen::VectorXf &desired_robot_state,
                                   std::vector<Eigen::VectorXf> &control_params) const override;
                        void steer(const ompl::state::SimEnvObjectState* current_robot_state,
                                   const ompl::state::SimEnvObjectState* desired_robot_state,
                                   std::vector<::ompl::control::Control*>& controls) const override;
                        void steer(const ompl::state::SimEnvObjectState* current_robot_state,
                                   const ompl::state::SimEnvObjectState* desired_robot_state,
                                   const ompl::state::SimEnvWorldState* current_world_state,
                                   std::vector<Eigen::VectorXf>& control_params) const override;

                    private:
                        ompl::control::RampVelocityControlSpacePtr _control_space;
                        ompl::state::SimEnvObjectConfigurationSpacePtr _robot_space;
                        ompl::control::RampVelocityControl* _ramp_control;
                    };
                    typedef std::shared_ptr<RampComputer> RampComputerPtr;
                    typedef std::shared_ptr<const RampComputer> RampComputerConstPtr;
                    typedef std::weak_ptr<RampComputer> RampComputerWeakPtr;
                    typedef std::weak_ptr<const RampComputer> RampComputerWeakConstPtr;
                }
            }
    };
}
#endif //MANIPULATION_PLANNING_SUITE_RAMPCOMPUTER_H
