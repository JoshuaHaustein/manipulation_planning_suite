//
// Created by joshua on 9/7/17.
//

#ifndef MANIPULATION_PLANNING_SUITE_LEARNEDORACLE_H
#define MANIPULATION_PLANNING_SUITE_LEARNEDORACLE_H

#include <mps/planner/pushing/oracle/Oracle.h>

namespace mps {
    namespace planner {
        namespace pushing {
            namespace oracle {
                class LearnedPipeOracle : public PushingOracle {
                   // TODO implement me
                public:
                    LearnedPipeOracle();
                    ~LearnedPipeOracle() override;

                    void prepareOracle(const Eigen::VectorXf &current_robot_state, const Eigen::VectorXf &current_obj_state,
                                       const Eigen::VectorXf &next_obj_state) override;

                    float predictPushability(const Eigen::VectorXf &current_obj_state,
                                             const Eigen::VectorXf &next_obj_state) override;

                    float predictFeasibility(const Eigen::VectorXf &current_robot_state,
                                             const Eigen::VectorXf &current_obj_state,
                                             const Eigen::VectorXf &next_obj_state) override;

                    void
                    predictAction(const Eigen::VectorXf &current_robot_state, const Eigen::VectorXf &current_obj_state,
                                  const Eigen::VectorXf &next_obj_state, Eigen::VectorXf &control) override;

                    void sampleFeasibleState(Eigen::VectorXf &new_robot_state,
                                             const Eigen::VectorXf &current_obj_state,
                                             const Eigen::VectorXf &next_obj_state) override;
                private:
                    /* Paths to pipes for communication with oracle */
                    const char *action_request_path;
                    const char *action_response_path;
                    const char *pushability_request_path;
                    const char *pushability_response_path;
                    const char *feasibility_request_path;
                    const char *feasibility_response_path;
                    const char *feasibility_sample_request_path;
                    const char *feasibility_sample_response_path;
                };
            }
        }
    }
}

#endif //MANIPULATION_PLANNING_SUITE_LEARNEDORACLE_H
