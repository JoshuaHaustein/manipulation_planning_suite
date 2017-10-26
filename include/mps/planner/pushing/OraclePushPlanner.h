//
// Created by joshua on 8/14/17.
//

#ifndef MANIPULATION_PLANNING_SUITE_ORACLEPUSHINGPLANNER_H
#define MANIPULATION_PLANNING_SUITE_ORACLEPUSHINGPLANNER_H

#include <map>
#include <sim_env/SimEnv.h>
#include <mps/planner/ompl/state/SimEnvState.h>
#include <mps/planner/ompl/state/SimEnvWorldStateDistanceMeasure.h>
#include <mps/planner/ompl/state/goal/ObjectsRelocationGoal.h>
#include <mps/planner/ompl/control/SimEnvStatePropagator.h>
#include <mps/planner/ompl/control/Interfaces.h>
#include <mps/planner/ompl/control/RampVelocityControl.h>
#include <mps/planner/pushing/algorithm/RRT.h>
#include <mps/planner/pushing/oracle/DataGenerator.h>

namespace mps {
    namespace planner {
        namespace pushing {
            /**
             * Definition of a push planning problem.
             */
            struct PlanningProblem {
                friend class OraclePushPlanner;
                enum OracleType {
                    Human = 0, Learned = 1
                };
                enum AlgorithmType {
                    Naive = 0, OracleRRT = 1, SliceOracleRRT = 2, CompleteSliceOracleRRT = 3
                };
                // world related parameters
                sim_env::WorldPtr world;
                sim_env::RobotPtr robot;
                sim_env::RobotVelocityControllerPtr robot_controller;
                // restrictions on the world
                std::map<std::string, Eigen::VectorXi> active_dofs;
                mps::planner::ompl::state::PlanningSceneBounds workspace_bounds;
                mps::planner::ompl::state::SimEnvValidityChecker::CollisionPolicy collision_policy;
                // time out for planner
                float planning_time_out;
                std::function<bool()> stopping_condition;
                // distance function
                std::map<std::string, float> object_weights;
                mps::planner::ompl::state::SimEnvWorldStateSpace::WeightMap weight_map;
                // parameters restricting action space
                mps::planner::ompl::control::RampVelocityControlSpace::ControlLimits control_limits;
                std::vector<mps::planner::ompl::control::RampVelocityControlSpace::ControlSubspace> control_subspaces;
                // parameters for semi-dynamic planning
                bool b_semi_dynamic;
                float t_max;
                // goal region
                std::vector<ompl::state::goal::RelocationGoalSpecification> relocation_goals;
                float goal_region_radius; // TODO should we have different radii for each object?
                // settings for control sampler
                OracleType oracle_type;
                AlgorithmType algorithm_type;
                unsigned int num_control_samples;
                float goal_bias;
                float robot_bias;
                float target_bias;
                unsigned int num_slice_neighbors;
                // flag whether to enable debug info
                bool debug;
                // TODO more parameters, like distance weights, workspace bounds, goal region

                /**
                 *  Constructor of a planning problem.
                 *  Initializes all non-essential parameters with default values.
                 */
                PlanningProblem(sim_env::WorldPtr world,
                                sim_env::RobotPtr robot,
                                sim_env::RobotVelocityControllerPtr controller,
                                const std::vector<ompl::state::goal::RelocationGoalSpecification>& goals);
                PlanningProblem(sim_env::WorldPtr world,
                                sim_env::RobotPtr robot,
                                sim_env::RobotVelocityControllerPtr controller,
                                const ompl::state::goal::RelocationGoalSpecification& goal);
            protected:
                // constructor that doesn't force you to provide mandatory arguments - used internally
                PlanningProblem();
            };


            /**
             * Solution to a push planning problem.
             */
            struct PlanningSolution {
                mps::planner::ompl::planning::essentials::PathPtr path;
                bool solved;
                mps::planner::pushing::algorithm::PlanningStatistics stats;
                PlanningSolution();
            };

            /**
             * This planner utilizes a kinodynamic RRT algorithm in combination
             * with an oracle to push a target object among movable and static obstacles into
             * some work space goal region. The planner performs a semi-dynamic search, that is
             * it plans on configuration space, but utilizes dynamic transitions between statically
             * stable states. As robot actions for these transitions, it utilizes RampVelocity
             * actions.
             */
            class OraclePushPlanner {
            public:
                OraclePushPlanner();
                ~OraclePushPlanner();

                bool setup(PlanningProblem& problem);
                bool solve(PlanningSolution& solution);
                // TODO should we move this playback function somewhere else?
                void playback(const PlanningSolution& solution, const std::function<bool()>& interrupt_callback=[](){return false;});
                void setSliceDrawer(algorithm::SliceDrawerInterfacePtr slice_drawer);
                void clearVisualizations();
                void generateData(const std::string& file_name,
                                  unsigned int num_samples,
                                  const std::string& annotation,
                                  bool deterministic);
                void dummyTest();
            private:
                bool _is_initialized;
                mps::planner::ompl::state::SimEnvWorldStateSpacePtr _state_space;
                mps::planner::ompl::control::RampVelocityControlSpacePtr _control_space;
                ::ompl::control::SpaceInformationPtr _space_information;
                mps::planner::ompl::state::SimEnvValidityCheckerPtr _validity_checker;
                mps::planner::ompl::control::SimEnvStatePropagatorPtr _state_propagator;
                PlanningProblem _planning_problem;
                mps::planner::pushing::algorithm::RearrangementRRTPtr _algorithm;
                mps::planner::pushing::algorithm::DebugDrawerPtr _debug_drawer;
                std::vector<float> _distance_weights;
                void prepareDistanceWeights();
                mps::planner::pushing::algorithm::RearrangementRRTPtr createAlgorithm(const PlanningProblem& pp) const;
//                ::ompl::control::DirectedControlSamplerPtr allocateDirectedControlSampler(const ::ompl::control::SpaceInformation* si);
                mps::planner::pushing::oracle::DataGeneratorPtr _data_generator;
            };

        }
    }
}

#endif //MANIPULATION_PLANNING_SUITE_ORACLEPUSHINGPLANNER_H
