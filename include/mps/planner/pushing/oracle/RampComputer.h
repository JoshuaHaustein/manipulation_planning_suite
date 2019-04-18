//
// Created by joshua on 9/8/17.
//

#ifndef MANIPULATION_PLANNING_SUITE_RAMPCOMPUTER_H
#define MANIPULATION_PLANNING_SUITE_RAMPCOMPUTER_H

#include <memory>
#include <mps/planner/ompl/control/RampVelocityControl.h>
#include <mps/planner/ompl/state/SimEnvState.h>
#include <mps/planner/pushing/oracle/Oracle.h>

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
                virtual void steer(const ompl::state::SimEnvObjectState* current_robot_state,
                    const ompl::state::SimEnvObjectState* desired_robot_state,
                    std::vector<::ompl::control::Control*>& controls) const = 0;
                virtual void steer(const ompl::state::SimEnvWorldState* current_state,
                    const ompl::state::SimEnvObjectState* desired_robot_state,
                    std::vector<::ompl::control::Control*>& controls) const = 0;
                /**
                         *  Compute a single control steering the robot from current_robot_state 
                         * to target_robot_state (as far as possible within one control).
                         */
                virtual void steer(const Eigen::VectorXf& current_robot_state,
                    const Eigen::VectorXf& target_robot_state,
                    ::ompl::control::Control* control) const = 0;
                /**
                         *  Compute a sequence of controls steering the robot from current_robot_state 
                         * to target_robot_state.
                         */
                virtual void steer(const Eigen::VectorXf& current_robot_state,
                    const Eigen::VectorXf& target_robot_state,
                    std::vector<::ompl::control::Control*>& controls) const = 0;

            private:
                void compute_controls(const Eigen::VectorXf& current_robot_state,
                    const Eigen::VectorXf& target_robot_state,
                    std::vector<Eigen::VectorXf>& control_params) const;
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
