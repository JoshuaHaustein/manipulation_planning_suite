#ifndef MPS_PLANNER_ORACLE_ORACLE_H
#define MPS_PLANNER_ORACLE_ORACLE_H

#include <Eigen/Core>

namespace mps {
    namespace planner {
        namespace oracle {
            class Oracle {
            public:
                /**
                 * Predicts the probability that the oracle is capable to provide
                 * an action to manipulate an object in current_obj_state such that it moves
                 * towards next_obj_state.
                 * @param current_obj_state - the current state of the object
                 * @param next_obj_state - the desired future state of the object
                 * @return propbability
                 */
                virtual float predictPushability(const Eigen::VectorXf& current_obj_state,
                                                 const Eigen::VectorXf& next_obj_state) = 0;
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
                virtual void sampleFeasibleState(const Eigen::VectorXf& new_robot_state,
                                            const Eigen::VectorXf& current_obj_state,
                                            const Eigen::VectorXf& next_obj_state) = 0;
                virtual ~Oracle() = 0;
            };
        }
    }
}

#endif // MPS_PLANNER_ORACLE_ORACLE_H
