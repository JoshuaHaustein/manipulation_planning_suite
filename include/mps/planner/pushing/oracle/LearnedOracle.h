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
                LearnedPipeOracle(const std::vector<sim_env::ObjectPtr>& objects, unsigned int robot_id);
                ~LearnedPipeOracle() override;

                void predictAction(const mps::planner::ompl::state::SimEnvWorldState* current_state,
                    const mps::planner::ompl::state::SimEnvWorldState* target_state,
                    const unsigned int& obj_id, ::ompl::control::Control* control) override;

                void samplePushingState(const mps::planner::ompl::state::SimEnvWorldState* current_state,
                    const mps::planner::ompl::state::SimEnvWorldState* next_state,
                    const unsigned int& obj_id,
                    mps::planner::ompl::state::SimEnvObjectState* new_robot_state) override;

            private:
                struct ObjectData {
                    float mass;
                    float inertia;
                    float mu;
                    float width;
                    float height;
                };
                /* Paths to pipes for communication with oracle */
                const char* _action_request_path;
                const char* _action_response_path;
                const char* _state_sample_request_path;
                const char* _state_sample_response_path;
                unsigned int _robot_id;
                std::vector<ObjectData> _object_data;
                Eigen::VectorXf _robot_state;
                Eigen::VectorXf _current_obj_state;
                Eigen::VectorXf _target_obj_state;
                Eigen::VectorXf _control;
            };
        }
    }
}
}

#endif //MANIPULATION_PLANNING_SUITE_LEARNEDORACLE_H
