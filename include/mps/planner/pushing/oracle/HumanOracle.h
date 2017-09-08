//
// Created by joshua on 9/7/17.
//

#ifndef MANIPULATION_PLANNING_SUITE_HUMANORACLE_H
#define MANIPULATION_PLANNING_SUITE_HUMANORACLE_H

#include <mps/planner/pushing/oracle/Oracle.h>

namespace mps {
    namespace planner {
        namespace pushing {
            namespace oracle {
                class HumanOracle : public PushingOracle {
                public:
                    HumanOracle();
                    ~HumanOracle();

                    void
                    prepareOracle(const Eigen::VectorXf &current_robot_state, const Eigen::VectorXf &current_obj_state,
                                  const Eigen::VectorXf &next_obj_state) override;

                    float predictPushability(const Eigen::VectorXf &current_obj_state,
                                             const Eigen::VectorXf &next_obj_state) override;

                    float predictFeasibility(const Eigen::VectorXf &current_robot_state,
                                             const Eigen::VectorXf &current_obj_state,
                                             const Eigen::VectorXf &next_obj_state) override;

                    void
                    predictAction(const Eigen::VectorXf &current_robot_state, const Eigen::VectorXf &current_obj_state,
                                  const Eigen::VectorXf &next_obj_state, Eigen::VectorXf &control) override;

                    void sampleFeasibleState(const Eigen::VectorXf &new_robot_state,
                                             const Eigen::VectorXf &current_obj_state,
                                             const Eigen::VectorXf &next_obj_state) override;

                };
                typedef std::shared_ptr<HumanOracle> HumanOraclePtr;
            }
        }
    }
}
#endif //MANIPULATION_PLANNING_SUITE_HUMANORACLE_H
