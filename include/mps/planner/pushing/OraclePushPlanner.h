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
#include <mps/planner/pushing/oracle/ElasticBandRampComputer.h>
#include <mps/planner/pushing/oracle/Oracle.h>

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

                enum LocalPlanner {
                    Line = 0, PotentialField = 1
                };

                enum AlgorithmType {
                    Naive = 0, OracleRRT = 1, SliceOracleRRT = 2, GNATSamplingSliceOracleRRT = 3,
                    SemanticGNATSamplingSliceOracleRRT = 4, HybridActionRRT = 5
                };

                enum ShortcutType {
                    NaiveShortcut = 0, OracleShortcut = 1, NoShortcut = 2, LocalShortcut = 3, LocalOracleShortcut = 4
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
                // settings for control sampler
                OracleType oracle_type;
                AlgorithmType algorithm_type;
                LocalPlanner local_planner_type;
                ShortcutType shortcut_type;
                float max_shortcut_time;
                unsigned int num_control_samples;
                float goal_bias;
                float robot_bias;
                float target_bias;
                float action_noise;
                float state_noise;
                bool do_slice_ball_projection;
                float min_state_distance;
                // flag whether to enable debug info
                bool debug;
                float sdf_resolution;
                float sdf_error_threshold;
                // TODO more parameters, like distance weights, goal region

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
                void playback(const PlanningSolution& solution,
                              const std::function<bool()>& interrupt_callback=[](){return false;},
                              bool force_synch=false);
                void setSliceDrawer(algorithm::SliceDrawerInterfacePtr slice_drawer);
                void renderSDF(float resolution);
                void clearVisualizations();
                void generateData(const std::string& file_name,
                                  unsigned int num_samples,
                                  const std::string& annotation,
                                  bool deterministic);
                void evaluateOracle(mps::planner::ompl::state::goal::RelocationGoalSpecification goal,
                                    const std::string& file_name,
                                    unsigned int num_samples,
                                    const std::string& annotation);
                void dummyTest();

                /***
                 * Verifies that the provided solutions is a solution to the current PlanningProblem.
                 * Returns false if the provided solution is not a reproducable solution,
                 * i.e. forward propagating it does not lead to a goal state while satisfying 
                 * all collision constraints.
                 */
                bool verifySolution(PlanningSolution& solution);

                /**
                 * This function allows to evaluate the oracle used by the current planning algorithm.
                 * If the planner is not setup or does not use an oracle,
                 * this function will immediately return. If the planner is setup and
                 * utilizes an oracle, this function will ask this oracle to provide a control
                 * that either moves the robot to its target state, the robot to a pushing state, or attempts to push an object.
                 * @param goal - a relocation goal specification. If the name of the object is the robot,
                 *              the oracle will be asked to provide a control to move the robot to the
                 *              desired state.
                 *              If the object name is an object, the oracle will be asked to
                 *              provide a control to either move to a pushing state or to push this object to the desired state.
                 * @param approach - if true and the target is an object, the returned control will move the robot 
                 *                  to a pushing state, else if the target is an object, it will push the object, else 
                 *                  the parameter has no effect
                 * @return a path containing the sequence of controls provided by the oracle. Nullptr in case of failure
                 */
                ompl::planning::essentials::PathPtr testOracle(const ompl::state::goal::RelocationGoalSpecification& goal, bool approach) const;
            private:
                bool _is_initialized;
                mps::planner::ompl::state::SimEnvWorldStateSpacePtr _state_space;
                mps::planner::ompl::control::RampVelocityControlSpacePtr _control_space;
                ::ompl::control::SpaceInformationPtr _space_information;
                mps::planner::ompl::state::SimEnvValidityCheckerPtr _validity_checker;
                mps::planner::ompl::control::SimEnvStatePropagatorPtr _state_propagator;
                PlanningProblem _planning_problem;
                mps::planner::pushing::algorithm::RearrangementRRTPtr _algorithm;
                mps::planner::pushing::algorithm::ShortcutterPtr _shortcutter;
                mps::planner::pushing::oracle::ElasticBandRampComputerPtr _eb_computer;
                mps::planner::pushing::algorithm::DebugDrawerPtr _rrt_debug_drawer;
                mps::planner::pushing::oracle::EBDebugDrawerPtr _eb_debug_drawer;
                std::vector<float> _distance_weights;
                void prepareDistanceWeights();
                void createAlgorithm(); // sets _algorithm and _shortcutter
                void createShortcutAlgorithm(oracle::PushingOraclePtr pushing_oracle,
                                             oracle::RobotOraclePtr robot_oracle);
//                ::ompl::control::DirectedControlSamplerPtr allocateDirectedControlSampler(const ::ompl::control::SpaceInformation* si);
                mps::planner::pushing::oracle::DataGeneratorPtr _data_generator;
                ompl::state::goal::ObjectsRelocationGoalPtr _goal_region;
            };

        }
    }
}

#endif //MANIPULATION_PLANNING_SUITE_ORACLEPUSHINGPLANNER_H
