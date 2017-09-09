//
// Created by joshua on 8/14/17.
//

#ifndef MANIPULATION_PLANNING_SUITE_ORACLEPUSHINGPLANNER_H
#define MANIPULATION_PLANNING_SUITE_ORACLEPUSHINGPLANNER_H

#include <map>
#include <sim_env/SimEnv.h>
#include <mps/planner/ompl/state/SimEnvState.h>
#include <mps/planner/ompl/control/SimEnvStatePropagator.h>
#include <mps/planner/ompl/control/Interfaces.h>
#include <mps/planner/ompl/control/RampVelocityControl.h>
#include <mps/planner/pushing/PushPlannerDistanceMeasure.h>
#include <mps/planner/pushing/algorithm/RRT.h>

namespace mps {
    namespace planner {
        namespace pushing {

            /**
             * Definition of a push planning problem.
             */
            struct PlanningProblem {
                friend class OraclePushPlanner;
                enum OracleType {
                    None = 0, Human = 1, Learned = 2
                };
                // world related parameters
                sim_env::WorldPtr world;
                sim_env::RobotPtr robot;
                sim_env::RobotVelocityControllerPtr robot_controller;
                sim_env::ObjectPtr target_object;
                // restrictions on the world
                std::map<std::string, Eigen::VectorXi> active_dofs;
                mps::planner::ompl::state::PlanningSceneBounds workspace_bounds;
                // time out for planner
                float planning_time_out;
                std::function<bool()> stopping_condition;
                // distance function
                std::map<std::string, float> object_weights;
                mps::planner::ompl::state::SimEnvWorldStateSpace::WeightMap weight_map;
                // parameters restricting action space
                mps::planner::ompl::control::RampVelocityControlSpace::ControlLimits control_limits;
                // parameters for semi-dynamic planning
                bool b_semi_dynamic;
                float t_max;
                // goal region
                Eigen::Vector3f goal_position;
                float goal_region_radius;
                // settings for control sampler
                OracleType oracle_type;
                unsigned int num_control_samples;
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
                                sim_env::ObjectPtr target_object,
                                const Eigen::Vector3f& goal_position);
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
                void playback(const PlanningSolution& solution);
                void clearVisualizations();
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
                mps::planner::pushing::algorithm::RearrangementRRT::DebugDrawerPtr _debug_drawer;
                mps::planner::ompl::state::SimEnvValidityChecker::CollisionPolicy _collision_policy;
                std::vector<float> _distance_weights;
                void prepareDistanceWeights();
                void prepareCollisionPolicy();
//                ::ompl::control::DirectedControlSamplerPtr allocateDirectedControlSampler(const ::ompl::control::SpaceInformation* si);
            };
        }
    }
}

#endif //MANIPULATION_PLANNING_SUITE_ORACLEPUSHINGPLANNER_H
