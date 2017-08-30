//
// Created by joshua on 8/14/17.
//

#ifndef MANIPULATION_PLANNING_SUITE_ORACLEPUSHINGPLANNER_H
#define MANIPULATION_PLANNING_SUITE_ORACLEPUSHINGPLANNER_H

#include <map>
#include <sim_env/SimEnv.h>
#include <mps/planner/ompl/state/SimEnvState.h>
//#include <mps/planner/ompl/state/goal/CircularWorkSpaceRegion.h>
#include <mps/planner/ompl/control/SimEnvStatePropagator.h>
#include <mps/planner/ompl/control/Interfaces.h>
#include <mps/planner/ompl/control/RampVelocityControl.h>
#include <mps/planner/pushing/PushPlannerDistanceMeasure.h>

namespace mps {
    namespace planner {
        namespace pushing {

            /**
             * Definition of a push planning problem.
             */
            struct PlanningProblem {
                friend class OraclePushPlanner;
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
                bool use_oracle;
                unsigned int num_control_samples;
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
                // TODO some encoding of the solution, probably a sequence of state, action pairs
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
                void dummyTest();
            private:
                bool _is_initialized;
                mps::planner::ompl::state::SimEnvWorldStateSpacePtr _state_space;
                mps::planner::ompl::control::RampVelocityControlSpacePtr _control_space;
                ::ompl::control::SpaceInformationPtr _space_information;
                mps::planner::ompl::state::SimEnvValidityCheckerPtr _validity_checker;
                mps::planner::ompl::control::SimEnvStatePropagatorPtr _state_propagator;
                PushPlannerDistanceMeasurePtr _distance_measure;
                PlanningProblem _planning_problem;
                PlanningSolution _planning_solution;

                void prepareDistanceWeights(std::vector<float>& weights);
                ::ompl::control::DirectedControlSamplerPtr allocateDirectedControlSampler(const ::ompl::control::SpaceInformation* si);
            };
        }
    }
}

#endif //MANIPULATION_PLANNING_SUITE_ORACLEPUSHINGPLANNER_H
