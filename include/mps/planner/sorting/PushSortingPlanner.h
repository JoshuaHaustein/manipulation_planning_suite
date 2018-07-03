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
                    DeterministicMCTS = 0, NonDetereministicMCTS = 1
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
                // std::vector<ompl::state::goal::RelocationGoalSpecification> relocation_goals;
                // settings for control sampler
                ValueFunctionType value_fn_type;
                AlgorithmType algorithm_type;
                float max_shortcut_time;
                unsigned int num_control_samples;
                // flag whether to enable debug info
                bool debug;

                /**
                 *  Constructor of a planning problem.
                 *  Initializes all non-essential parameters with default values.
                 */
                PlanningProblem(sim_env::WorldPtr world,
                                sim_env::RobotPtr robot,
                                sim_env::RobotVelocityControllerPtr controller);
                PlanningProblem(const PlanningProblem& other);
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
