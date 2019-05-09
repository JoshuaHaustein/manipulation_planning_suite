//
// Created by joshua on 9/29/17.
//

#ifndef MANIPULATION_PLANNING_SUITE_DATAGENERATOR_H
#define MANIPULATION_PLANNING_SUITE_DATAGENERATOR_H

#include <mps/planner/ompl/control/RampVelocityControl.h>
#include <mps/planner/ompl/control/SimEnvStatePropagator.h>
#include <mps/planner/ompl/state/SimEnvState.h>
#include <mps/planner/ompl/state/goal/ObjectsRelocationGoal.h>
#include <mps/planner/pushing/oracle/OracleControlSampler.h>

namespace mps {
namespace planner {
    namespace pushing {
        namespace oracle {
            class DataGenerator {
            public:
                struct Parameters {
                    float obj_position_sigma;
                    float obj_orientation_sigma;
                    float mass_sigma;
                    float friction_sigma;
                    Parameters();
                };
                DataGenerator(::ompl::control::SpaceInformationPtr space_info,
                    mps::planner::ompl::control::SimEnvStatePropagatorPtr state_prop,
                    sim_env::WorldPtr world,
                    const std::string& robot_name,
                    const std::string& obj_name,
                    const Parameters& params = Parameters());
                ~DataGenerator();
                void generateData(const std::string& file_name,
                    unsigned int num_samples,
                    const std::string& annotation,
                    bool deterministic = false,
                    unsigned int num_noise_samples = 10);
                void evaluateOracle(mps::planner::ompl::state::goal::RelocationGoalSpecification goal,
                    mps::planner::pushing::oracle::OracleControlSamplerPtr oracle_sampler,
                    mps::planner::ompl::state::SimEnvWorldStateSpacePtr state_space,
                    const std::string& file_name,
                    unsigned int num_samples,
                    const std::string& annotation);

            private:
                struct ObjectDynamics {
                    float mass;
                    float friction_coeff;
                };
                sim_env::WorldPtr _world;
                sim_env::RobotPtr _robot;
                sim_env::ObjectPtr _object;
                unsigned int _object_id;
                unsigned int _robot_id;
                ::ompl::control::SpaceInformationPtr _space_information;
                mps::planner::ompl::control::SimEnvStatePropagatorPtr _state_propagator;
                float _max_distance;
                Eigen::VectorXf _state_sigma;
                float _mass_stddev;
                float _friction_stddev;
                ObjectDynamics _original_dynamics;

                bool sampleValidState(::ompl::base::State* state,
                    ::ompl::base::StateSamplerPtr state_sampler,
                    ::ompl::base::StateValidityCheckerPtr validity_checker);
                void applyNoise(const ::ompl::base::State* mean_state, ::ompl::base::State* noisy_state);
                void modifyDynamics();
                void restoreDynamics();
                void computeMaxDistance();
                bool objectMoved(const ::ompl::base::State* initial_state,
                    const ::ompl::base::State* result_state) const;
            };
            typedef std::shared_ptr<DataGenerator> DataGeneratorPtr;
        }
    }
}
}
#endif //MANIPULATION_PLANNING_SUITE_DATAGENERATOR_H
