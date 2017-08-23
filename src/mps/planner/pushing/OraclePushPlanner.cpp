//
// Created by joshua on 8/21/17.
//

#include <mps/planner/pushing/OraclePushPlanner.h>
#include <thread>
#include <chrono>

using namespace mps::planner::pushing;
namespace mps_state = mps::planner::ompl::state;
namespace mps_control = mps::planner::ompl::control;

PlanningProblem::PlanningProblem() {
}

PlanningProblem::PlanningProblem(sim_env::WorldPtr world, sim_env::RobotPtr robot,
                                 sim_env::RobotVelocityControllerPtr controller,
                                 sim_env::ObjectPtr target_object):
        world(world), robot(robot), robot_controller(controller), target_object(target_object)
{

    planning_time_out = 60.0f;
    b_semi_dynamic = true;
    t_max = 8.0f;
    workspace_bounds.x_limits[0] = std::numeric_limits<float>::lowest();
    workspace_bounds.x_limits[1] = std::numeric_limits<float>::max();
    workspace_bounds.y_limits[0] = std::numeric_limits<float>::lowest();
    workspace_bounds.y_limits[1] = std::numeric_limits<float>::max();
    workspace_bounds.z_limits[0] = std::numeric_limits<float>::lowest();
    workspace_bounds.z_limits[1] = std::numeric_limits<float>::max();
    workspace_bounds.max_rotation_vel = 10.0f; // arbitrary maximum velocity in rad/s
    workspace_bounds.max_velocity = 30.0f; // arbitrary maximum velocity in m/s
}

OraclePushPlanner::OraclePushPlanner() {
    _is_initialized = false;
}

OraclePushPlanner::~OraclePushPlanner() = default;

bool OraclePushPlanner::setup(PlanningProblem& problem) {
    // first delete any previous instances
    _space_information.reset();
    _state_space.reset();
    _control_space.reset();
    _validity_checker.reset();
    _state_propagator.reset();
    _planning_problem = problem;
    _planning_solution = PlanningSolution();
    // now create new instances
    _state_space = std::make_shared<mps_state::SimEnvWorldStateSpace>(_planning_problem.world,
                                                                      _planning_problem.workspace_bounds,
                                                                      _planning_problem.b_semi_dynamic,
                                                                      _planning_problem.weight_map);
    _control_space =
            std::make_shared<mps_control::RampVelocityControlSpace>(_state_space,
                                                                    problem.control_limits);
    _space_information =
            std::make_shared<::ompl::control::SpaceInformation>(_state_space, _control_space);
    _space_information->setPropagationStepSize(1.0); // NOT USED
    _space_information->setMinMaxControlDuration(1, 1); // NOT USED
    _validity_checker =
            std::make_shared<mps::planner::ompl::state::SimEnvValidityChecker>(_space_information,
                                                                               _planning_problem.world);
    _space_information->setStateValidityChecker(_validity_checker);
    _state_propagator =
            std::make_shared<mps::planner::ompl::control::SimEnvStatePropagator>(_space_information,
                                                                                 _planning_problem.world,
                                                                                 _planning_problem.robot_controller,
                                                                                 _planning_problem.b_semi_dynamic,
                                                                                 _planning_problem.t_max);
    _space_information->setStatePropagator(_state_propagator);
    _space_information->setup();
    // TODO create planning algorithm
//    _rrt = std::make_shared<mps::planner::algorithm::RRT>(_space_information);
    _is_initialized = true;
    return _is_initialized;
}

bool OraclePushPlanner::solve(PlanningSolution& solution) {
    // TODO run planning algorithm, extract solution and return it
    return false;
}

void OraclePushPlanner::dummyTest() {
    // TODO can put manual tests here
    ::ompl::base::StateSamplerPtr state_sampler = _space_information->allocStateSampler();
    ::ompl::base::State* state = _space_information->allocState();
    for (unsigned int i = 0; i < 10; ++i) {
        state_sampler->sampleUniform(state);
        auto* world_state = state->as<mps_state::SimEnvWorldState>();
        _state_space->setToState(_planning_problem.world, world_state);
        std::chrono::milliseconds sleeping_time(500);
        std::this_thread::sleep_for(sleeping_time);
    }
    _space_information->freeState(state);
}
