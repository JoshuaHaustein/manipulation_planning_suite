//
// Created by joshua on 9/7/17.
//

#ifndef MANIPULATION_PLANNING_SUITE_HUMANORACLE_H
#define MANIPULATION_PLANNING_SUITE_HUMANORACLE_H

#include <mps/planner/pushing/oracle/Oracle.h>
#include <mps/planner/pushing/oracle/RampComputer.h>
#include <mps/planner/util/Random.h>
#include <sim_env/SimEnv.h>

namespace mps {
namespace planner {
    namespace pushing {
        namespace oracle {
            /**
             * Hand designed pushing oracle for holonomic 2D robots and objects in SE(2).
             */
            class HumanOracle : public PushingOracle {
            public:
                struct Parameters {
                    Parameters(); // inits values suitable for rigid_fingers hand
                    Eigen::Vector3f robot_pose; // ideal robot pose relative to the object, if it should be pushed along the x-axis
                    float alpha; // size of arc (radians) on which the robot may be placed relative to the pushed object
                    // alpha = 0 -> the robot needs to be on the opposite side of the pushing direction
                    // alpha = pi / 2 -> the robot may also be placed next to the object facing orhogonal to the pushing direction
                    // alpha = pi -> the robot can be placed anywhere surrounding the object, i.e. it fully cages the object
                    float sig_trans; // variance when sampling pushing translation (for policy)
                    float sig_rot; // variance when sampling pushing rotation (for policy)
                };
                // struct Parameters {
                //     friend class HumanOracle;
                //     Eigen::Matrix3f pushability_covariance;
                //     float optimal_push_distance;
                //     float push_distance_tolerance;
                //     float push_angle_tolerance;
                //     Parameters();
                //     void computeInverses();

                // protected:
                //     Eigen::Matrix3f _inv_pushability_covariance;
                // };

                explicit HumanOracle(RobotOraclePtr robot_oracle,
                    unsigned int robot_id,
                    const std::vector<sim_env::ObjectPtr>& objects,
                    const Parameters& params = Parameters());
                ~HumanOracle() override;

                void predictAction(const mps::planner::ompl::state::SimEnvWorldState* current_state,
                    const mps::planner::ompl::state::SimEnvWorldState* target_state,
                    const unsigned int& obj_id, ::ompl::control::Control* control) override;

                void samplePushingState(const mps::planner::ompl::state::SimEnvWorldState* current_state,
                    const mps::planner::ompl::state::SimEnvWorldState* next_state,
                    const unsigned int& obj_id,
                    mps::planner::ompl::state::SimEnvObjectState* new_robot_state) override;

            private:
                Parameters _params;
                RobotOraclePtr _robot_steerer;
                unsigned int _robot_id;
                ::ompl::RNGPtr _rng;
                Eigen::VectorXf _next_obj_state;
                Eigen::VectorXf _current_obj_state;
                Eigen::VectorXf _robot_state;
                Eigen::VectorXf _next_robot_state;
                // Takes the first three components of state and ref and computes state - ref
                // The third component is assumed to be an angle in range [-pi, pi], hence the shortest
                Eigen::Vector3f relativeSE2(const Eigen::VectorXf& state, const Eigen::VectorXf& ref);
            };
            typedef std::shared_ptr<HumanOracle> HumanOraclePtr;
        }
    }
}
}
#endif //MANIPULATION_PLANNING_SUITE_HUMANORACLE_H
