#ifndef MPS_PLANNER_ORACLE_ORACLE_H
#define MPS_PLANNER_ORACLE_ORACLE_H

#include <Eigen/Core>
#include <memory>
#include <mps/planner/ompl/state/SimEnvState.h>
#include <mps/planner/util/Time.h>
#include <ompl/control/Control.h>
#include <vector>

namespace mps {
namespace planner {
    namespace pushing {
        namespace oracle {
            class PushingOracle {
            public:
                virtual ~PushingOracle() = 0;

                /**
                     * Predicts what action should be taken in the current robot state
                     * in order to move the object from its current state to the desired target state.
                     * @param current_state - current state
                     * @param next_state - desired next state
                     * @param obj_id - object id identifying the object to be pushed
                     * @param control - output control that is directed towards next_state
                     */
                virtual void predictAction(const mps::planner::ompl::state::SimEnvWorldState* current_state,
                    const mps::planner::ompl::state::SimEnvWorldState* target_state,
                    const unsigned int& obj_id, ::ompl::control::Control* control)
                    = 0;

                /**
                     * Samples a robot state that is likely to be feasible for applying this oracle.
                     * In other words, this function samples from the feasibility distribution.
                     * @param current_obj_state - current object state
                     * @param next_obj_state - desired future object state
                     * @param obj_id - object id identifying the object to be pushed
                     * @param new_robot_state  - current robot state
                     */
                virtual void samplePushingState(const mps::planner::ompl::state::SimEnvWorldState* current_state,
                    const mps::planner::ompl::state::SimEnvWorldState* next_state,
                    const unsigned int& obj_id,
                    mps::planner::ompl::state::SimEnvObjectState* new_robot_state)
                    = 0;

                /**
                    * Optional timer for adding external cpu time to clock
                    */
                std::shared_ptr<mps::planner::util::time::Timer> timer;
            };

            typedef std::shared_ptr<PushingOracle> PushingOraclePtr;
            typedef std::shared_ptr<const PushingOracle> PushingOracleConstPtr;
            typedef std::weak_ptr<PushingOracle> PushingOracleWeakPtr;
            typedef std::weak_ptr<const PushingOracle> PushingOracleWeakConstPtr;

            class RobotOracle {
            public:
                virtual ~RobotOracle() = 0;
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

                /**
                     * Computes a sequence of controls that attempt to move the robot from
                     * the current state to the desired state given the current state of the world.
                     * The control sequences is parameterized and returned in the vector control_params.
                     */
                // virtual void steer(const ompl::state::SimEnvObjectState* current_robot_state,
                //     const ompl::state::SimEnvObjectState* desired_robot_state,
                //     const ompl::state::SimEnvWorldState* current_world_state,
                //     std::vector<Eigen::VectorXf>& control_params) const = 0;
            };

            typedef std::shared_ptr<RobotOracle> RobotOraclePtr;
            typedef std::shared_ptr<const RobotOracle> RobotOracleConstPtr;
            typedef std::weak_ptr<RobotOracle> RobotOracleWeakPtr;
            typedef std::weak_ptr<const RobotOracle> RobotOracleWeakConstPtr;
        }
    }
}
}

#endif // MPS_PLANNER_ORACLE_ORACLE_H
