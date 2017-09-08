//
// Created by joshua on 9/8/17.
//

#ifndef MANIPULATION_PLANNING_SUITE_RAMPCOMPUTER_H
#define MANIPULATION_PLANNING_SUITE_RAMPCOMPUTER_H

#include <mps/planner/pushing/oracle/Oracle.h>
#include <mps/planner/ompl/control/RampVelocityControl.h>
#include <mps/planner/ompl/state/SimEnvState.h>

namespace mps {
    namespace planner {
            namespace pushing {
                namespace oracle {
                    class RampComputer : public RobotOracle {
                    public:
                        RampComputer(ompl::state::SimEnvObjectConfigurationSpacePtr robot_space,
                                     ompl::control::RampVelocityControlSpacePtr control_space);
                        ~RampComputer() override;

                        void steer(const Eigen::VectorXf &current_robot_state,
                                   const Eigen::VectorXf &desired_robot_state,
                                   std::vector<Eigen::VectorXf> &control_params) const override;

                    private:
                        ompl::control::RampVelocityControlSpacePtr _control_space;
                        ompl::state::SimEnvObjectConfigurationSpacePtr _robot_space;
                        ompl::control::RampVelocityControl* _ramp_control;
                    };
                }
            }
    };
}
#endif //MANIPULATION_PLANNING_SUITE_RAMPCOMPUTER_H
