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
                public:
                    struct ObjectData {
                        float mass;
                        float inertia;
                        float width;
                        float height;
                        float mu;
                    };

                    LearnedPipeOracle(const std::vector<ObjectData>& object_data);
                    ~LearnedPipeOracle() override;

                    void setObjectData(const std::vector<ObjectData>& object_data);

                    float predictPushability(const Eigen::VectorXf &current_obj_state,
                                             const Eigen::VectorXf &next_obj_state,
                                             const unsigned int& obj_id) override;

                    void projectToPushability(const Eigen::VectorXf& current_obj_state,
                                              const Eigen::VectorXf& next_obj_state,
                                              const float& num_std,
                                              const unsigned int& obj_id,
                                              Eigen::VectorXf& output) override;

                    float predictFeasibility(const Eigen::VectorXf &current_robot_state,
                                             const Eigen::VectorXf &current_obj_state,
                                             const Eigen::VectorXf &next_obj_state,
                                             const unsigned int& obj_id) override;

                    void predictAction(const Eigen::VectorXf &current_robot_state,
                                       const Eigen::VectorXf &current_obj_state,
                                       const Eigen::VectorXf &next_obj_state,
                                       const unsigned int& obj_id,
                                       Eigen::VectorXf &control) override;

                    void sampleFeasibleState(const Eigen::VectorXf &current_obj_state,
                                             const Eigen::VectorXf &next_obj_state,
                                             const unsigned int& obj_id,
                                             Eigen::VectorXf &new_robot_state) override;
                private:
                    /* Paths to pipes for communication with oracle */
                    const char *_action_request_path;
                    const char *_action_response_path;
                    const char *_pushability_request_path;
                    const char *_pushability_response_path;
                    const char *_feasibility_request_path;
                    const char *_feasibility_response_path;
                    const char *_feasibility_sample_request_path;
                    const char *_feasibility_sample_response_path;

                    std::vector<ObjectData> _object_data;
                };
            }
        }
    }
}

#endif //MANIPULATION_PLANNING_SUITE_LEARNEDORACLE_H
