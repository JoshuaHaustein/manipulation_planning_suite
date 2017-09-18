#ifndef MPS_PLANNER_ORACLE_ORACLE_H
#define MPS_PLANNER_ORACLE_ORACLE_H

#include <Eigen/Core>
#include <memory>
#include <vector>

namespace mps {
    namespace planner {
        namespace pushing {
            namespace oracle {
                class PushingOracle {
                public:
                    virtual ~PushingOracle() = 0;
                    // TODO can we make all this functions const?
                    // TODO it would make more sense if these function take SimEnvObjectState* as input rather than Eigen::VectorXf
                    // TODO similarly, it would make sense to pass VelocityControl* as controls
                    /**
                     * Notifies the oracle that any sequence of predictPushability, predictFeasibility, predictAction
                     * requests with the given arguments will follow. This allows the oracle to compute data that is
                     * required for all of these queries.
                     * @param current_robot_state - current state of the robot
                     * @param current_obj_state - current state of the object
                     * @param next_obj_state - next desired object state
                     */
                    virtual void prepareOracle(const Eigen::VectorXf& current_robot_state,
                                               const Eigen::VectorXf& current_obj_state,
                                               const Eigen::VectorXf& next_obj_state) = 0;
                    /**
                     * Predicts how well the oracle is capable to provide
                     * an action to manipulate an object in current_obj_state such that it moves
                     * towards next_obj_state.
                     * @param current_obj_state - the current state of the object
                     * @param next_obj_state - the desired future state of the object
                     * @return propbability
                     */
                    virtual float predictPushability(const Eigen::VectorXf& current_obj_state,
                                                     const Eigen::VectorXf& next_obj_state) = 0;

                    /**
                     * Projects the given next_obj_state on to a state output such that the direction
                     * from current_obj_state to output is the same as from current_obj_state to next_obj_state
                     * and the output state is within num_std * standard deviations of pushability away from current_obj_state.
                     * @param current_obj_state - current object state
                     * @param next_obj_state - next object state
                     * @param output - projected state
                     * @param num_std - multiplier denoting how many stds away from current_obj_state output state can be
                     */
                    virtual void projectToPushability(const Eigen::VectorXf& current_obj_state,
                                                    const Eigen::VectorXf& next_obj_state,
                                                    Eigen::VectorXf& output, float num_std=1.0f) = 0;

                    /**
                     * Predicts the feasibility of the oracle given the current robot state, object state
                     * and the desired future object state. Another interpretation of the feasibility
                     * is the probability that the oracle can provide an action achieving the desired object
                     * state space given the current robot state.
                     * @param current_robot_state
                     * @param current_obj_state
                     * @param next_obj_state
                     * @return feasibility
                     */
                    virtual float predictFeasibility(const Eigen::VectorXf& current_robot_state,
                                                     const Eigen::VectorXf& current_obj_state,
                                                     const Eigen::VectorXf& next_obj_state) = 0;

                    /**
                     * Predicts what action should be taken in the current robot state
                     * in order to move the object from its current state to the desired future state.
                     * @param current_state - current state
                     * @param next_state - desired next state
                     * @param control - output control that is directed towards next_state
                     */
                    virtual void predictAction(const Eigen::VectorXf& current_robot_state,
                                               const Eigen::VectorXf& current_obj_state,
                                               const Eigen::VectorXf& next_obj_state,
                                               Eigen::VectorXf& control) = 0;

                    /**
                     * Samples a robot state that is likely to be feasible for applying this oracle.
                     * In other words, this function samples from the feasibility distribution.
                     * @param new_robot_state  - current robot state
                     * @param current_obj_state - current object state
                     * @param next_obj_state - desired future object state
                     */
                    virtual void sampleFeasibleState(Eigen::VectorXf& new_robot_state,
                                                     const Eigen::VectorXf& current_obj_state,
                                                     const Eigen::VectorXf& next_obj_state) = 0;
                };

                typedef std::shared_ptr<PushingOracle> PushingOraclePtr;
                typedef std::shared_ptr<const PushingOracle> PushingOracleConstPtr;
                typedef std::weak_ptr<PushingOracle> PushingOracleWeakPtr;
                typedef std::weak_ptr<const PushingOracle> PushingOracleWeakConstPtr;

                class RobotOracle {
                public:
                    virtual ~RobotOracle() = 0;
                    // TODO same as for the pushing oracle, it would make sense to use SimEnvObjectState* here
                    virtual void steer(const Eigen::VectorXf& current_robot_state,
                                       const Eigen::VectorXf& desired_robot_state,
                                       std::vector<Eigen::VectorXf>& control_params) const = 0;
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
