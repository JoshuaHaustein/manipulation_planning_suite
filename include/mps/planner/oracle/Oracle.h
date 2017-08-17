#ifndef MPS_PLANNER_ORACLE_ORACLE_H
#define MPS_PLANNER_ORACLE_ORACLE_H

#include <Eigen/Core>

namespace mps {
    namespace planner {
        namespace oracle {
            class Oracle {
            public:
                /**
                 * Predicts what action should be taken in current_state in order to reach the next_state.
                 * @param current_state - current state
                 * @param next_state - desired next state
                 * @param control - output control that is directed towards next_state
                 * @return a confidence value indicating how certain the oracle is about this prediction.
                 *         0 = maximum confidence; the larger, the less confidence.
                 */
                virtual float predictAction(const Eigen::VectorXf& current_state, const Eigen::VectorXf& next_state, Eigen::VectorXf& control) = 0;
                virtual ~Oracle() = 0;
            };
        }
    }
}

#endif // MPS_PLANNER_ORACLE_ORACLE_H
