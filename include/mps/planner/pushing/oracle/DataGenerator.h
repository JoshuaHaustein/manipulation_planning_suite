//
// Created by joshua on 9/29/17.
//

#ifndef MANIPULATION_PLANNING_SUITE_DATAGENERATOR_H
#define MANIPULATION_PLANNING_SUITE_DATAGENERATOR_H

#include <mps/planner/ompl/state/SimEnvState.h>
#include <mps/planner/ompl/control/RampVelocityControl.h>
#include <mps/planner/ompl/control/SimEnvStatePropagator.h>

namespace mps {
    namespace planner {
        namespace pushing {
            namespace oracle {
                class DataGenerator {
                public:
                    DataGenerator(::ompl::control::SpaceInformationPtr space_info,
                                  mps::planner::ompl::control::SimEnvStatePropagatorPtr state_prop,
                                  sim_env::WorldPtr world,
                                  const std::string& robot_name,
                                  const std::string& obj_name);
                    ~DataGenerator();
                    void generateData(const std::string& file_name,
                                      unsigned int num_samples,
                                      const std::string& header,
                                      bool deterministic=false,
                                      unsigned int num_noise_samples=10);
                private:
                    sim_env::WorldPtr _world;
                    sim_env::RobotPtr _robot;
                    sim_env::ObjectPtr _object;
                    unsigned int _object_id;
                    unsigned int _robot_id;
                    ::ompl::control::SpaceInformationPtr _space_information;
                    mps::planner::ompl::control::SimEnvStatePropagatorPtr _state_propagator;
                    float _max_distance;
                    Eigen::VectorXf _state_sigma;

                    bool sampleValidState(::ompl::base::State* state,
                                          ::ompl::base::StateSamplerPtr state_sampler,
                                          ::ompl::base::StateValidityCheckerPtr validity_checker);
                    void applyNoise(const ::ompl::base::State* mean_state, ::ompl::base::State* noisy_state);
                    void modifyDynamics();
                    void restoreDynamics();

                };
                typedef std::shared_ptr<DataGenerator> DataGeneratorPtr;
            }
        }
    }
}
#endif //MANIPULATION_PLANNING_SUITE_DATAGENERATOR_H
