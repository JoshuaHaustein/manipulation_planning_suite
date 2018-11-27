//
// Created by joshua on 7/02/18.
//

#ifndef MANIPULATION_PLANNING_SUITE_PUSHSORTINGPLANNER_H
#define MANIPULATION_PLANNING_SUITE_PUSHSORTINGPLANNER_H

#include <sim_env/SimEnv.h>
#include <mps/planner/ompl/state/SimEnvState.h>
#include <mps/planner/ompl/state/SimEnvWorldStateDistanceMeasure.h>
#include <mps/planner/ompl/control/SimEnvStatePropagator.h>
#include <mps/planner/ompl/control/Interfaces.h>
#include <mps/planner/ompl/control/RampVelocityControl.h>
#include <mps/planner/sorting/algorithm/MCTS.h>
// #include <mps/planner/pushing/oracle/Oracle.h>

namespace mps {
    namespace planner {
        namespace sorting {
            /**
             * Definition of an sort push planning problem.
             */
            struct PlanningProblem {
                friend class PushSortingPlanner;

                enum ValueFunctionType {
                    Entropy = 0, Learned = 1
                };

                enum AlgorithmType {
                    DeterministicMCTS = 0, NonDeterministicMCTS = 1
                };

                /// World related parameters, that are mandatory to be set manually ///
                sim_env::WorldPtr world;
                sim_env::RobotVelocityControllerPtr robot_controller;
                // The robot is initialized by calling init_robot()
                sim_env::RobotPtr robot;

                /// Parameters that are initialized with default values on construction ///
                float planning_time_out;
                float t_max; // maximum waiting time (semi dynamic planning)
                mps::planner::ompl::state::PlanningSceneBounds workspace_bounds;
                // parameters restricting action space
                std::vector<mps::planner::ompl::control::RampVelocityControlSpace::ControlSubspace> control_subspaces;
                // goal region
                std::map<std::string, unsigned int> sorting_groups; 
                // settings for control sampler
                ValueFunctionType value_fn_type;
                AlgorithmType algorithm_type;
                bool debug; // flag whether to enable debug info
                unsigned int num_control_samples;
                std::function<bool()> stopping_condition;
                mps::planner::ompl::state::SimEnvValidityChecker::CollisionPolicy collision_policy;
                
                /// Parameters initialized by init_control_limits() ///
                // std::map<std::string, Eigen::VectorXi> active_dofs; // restictions on active degrees of freedom
                mps::planner::ompl::control::RampVelocityControlSpace::ControlLimits control_limits;

                PlanningProblem();
                PlanningProblem(const PlanningProblem& other);

                /**
                 * Initialize all parameters that are dependent on the robot.
                 * The world and the robot_controller need to have been set before!
                 * @return true if initialization successful
                 */
                bool init_robot();
            };

            /**
             * Solution to a push planning problem.
             */
            struct PlanningSolution {
                mps::planner::ompl::planning::essentials::PathPtr path;
                bool solved;
                mps::planner::sorting::algorithm::PlanningStatistics stats;
                PlanningSolution();
            };

            /**
             * This class will serve as interface to your algorithms. It should set up everything
             * needed for planning and then run the selected algorithm. You will probably implement
             * different algorithms to compare, so decoupling the setup from the actual algorithm
             * is useful.
             */
            class PushSortingPlanner {
            public:
                PushSortingPlanner();
                ~PushSortingPlanner();

                /**
                 * Set the planner up with the given problem. Needs to be called before calling solve().
                 * This function initializes the state, control spaces and other data structures
                 * needed for planning.
                 * Returns true if setup successful, else false
                 */
                bool setup(PlanningProblem& problem);
                /**
                 * Runs the planner on the problem set up in the last call to setup(..).
                 * Returns true if planning was successful and saves the solution in the provided
                 * argument.
                 */
                bool solve(PlanningSolution& solution);
                // Play back a solution found by this planner
                void playback(const PlanningSolution& solution,
                              const std::function<bool()>& interrupt_callback=[](){return false;},
                              bool force_synch=false);

            private:
                bool _is_initialized;
                mps::planner::ompl::state::SimEnvWorldStateSpacePtr _state_space;
                mps::planner::ompl::control::RampVelocityControlSpacePtr _control_space;
                ::ompl::control::SpaceInformationPtr _space_information;
                mps::planner::ompl::state::SimEnvValidityCheckerPtr _validity_checker;
                mps::planner::ompl::control::SimEnvStatePropagatorPtr _state_propagator;
                PlanningProblem _planning_problem;
                mps::planner::sorting::algorithm::MCTSBasePtr _algorithm;
                // mps::planner::pushing::algorithm::DebugDrawerPtr _rrt_debug_drawer;
                // mps::planner::pushing::oracle::EBDebugDrawerPtr _eb_debug_drawer;
                void createAlgorithm(); // sets _algorithm and _shortcutter
            };

        }
    }
}

#endif //MANIPULATION_PLANNING_SUITE_ORACLEPUSHINGPLANNER_H
