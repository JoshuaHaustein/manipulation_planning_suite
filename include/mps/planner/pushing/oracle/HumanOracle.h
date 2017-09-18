//
// Created by joshua on 9/7/17.
//

#ifndef MANIPULATION_PLANNING_SUITE_HUMANORACLE_H
#define MANIPULATION_PLANNING_SUITE_HUMANORACLE_H

#include <mps/planner/pushing/oracle/Oracle.h>
#include <mps/planner/pushing/oracle/RampComputer.h>
#include <mps/planner/util/Random.h>

namespace mps {
    namespace planner {
        namespace pushing {
            namespace oracle {
                /**
                 * Hand designed pushing oracle.
                 * TODO: current implementation assumes object states are in SE(2)
                 * TODO: current implementation assumes ramp control actions
                 */
                class HumanOracle : public PushingOracle {
                public:
                    struct Parameters {
                        friend class HumanOracle;
                        Eigen::Matrix3f pushability_covariance;
                        float optimal_push_distance;
                        float push_distance_tolerance;
                        float push_angle_tolerance;
                        Parameters();
                        void computeInverses();
                    protected:
                        Eigen::Matrix3f _inv_pushability_covariance;
                    };

                    explicit HumanOracle(RampComputerPtr ramp_computer,
                                         const Parameters& params=Parameters());
                    ~HumanOracle() override;

                    void prepareOracle(const Eigen::VectorXf &current_robot_state,
                                       const Eigen::VectorXf &current_obj_state,
                                       const Eigen::VectorXf &next_obj_state) override;

                    float predictPushability(const Eigen::VectorXf &current_obj_state,
                                             const Eigen::VectorXf &next_obj_state) override;

                    void projectToPushability(const Eigen::VectorXf& current_obj_state,
                                              const Eigen::VectorXf& next_obj_state,
                                              Eigen::VectorXf& output, float num_std=1.0f) override;

                    float predictFeasibility(const Eigen::VectorXf &current_robot_state,
                                             const Eigen::VectorXf &current_obj_state,
                                             const Eigen::VectorXf &next_obj_state) override;

                    void predictAction(const Eigen::VectorXf &current_robot_state,
                                       const Eigen::VectorXf &current_obj_state,
                                       const Eigen::VectorXf &next_obj_state,
                                       Eigen::VectorXf &control) override;

                    void sampleFeasibleState(Eigen::VectorXf &new_robot_state,
                                             const Eigen::VectorXf &current_obj_state,
                                             const Eigen::VectorXf &next_obj_state) override;

                private:
                    Parameters _params;
                    RampComputerPtr _ramp_computer;
                    ::ompl::RNGPtr _rng;
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
