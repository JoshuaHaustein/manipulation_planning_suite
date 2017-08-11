#ifndef MPS_PLANNER_ORACLE_ORACLE_H
#define MPS_PLANNER_ORACLE_ORACLE_H

#include <Eigen/Core>

namespace mps {
    namespace planner {
        namespace oracle {
            class Oracle {
            public:
                virtual void predictAction(const Eigen::VectorXf& current_state, const Eigen::VectorXf& next_state, Eigen::VectorXf& control) = 0;
                virtual ~Oracle() = 0;
            };
        }
    }
}

#endif // MPS_PLANNER_ORACLE_ORACLE_H
