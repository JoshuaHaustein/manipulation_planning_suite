//
// Created by joshua on 8/21/17.
//

#include <mps/planner/pushing/OraclePushPlanner.h>
#include <mps/planner/util/Serialize.h>
#include <mps/planner/ompl/state/goal/ObjectRelocationGoal.h>
#include <thread>
#include <chrono>
#include <mps/planner/ompl/control/OracleControlSampler.h>
#include <mps/planner/ompl/control/NaiveControlSampler.h>
#include <mps/planner/util/Logging.h>


using namespace mps::planner::pushing;
namespace mps_state = mps::planner::ompl::state;
namespace mps_control = mps::planner::ompl::control;

PlanningProblem::PlanningProblem() :
        control_limits(Eigen::VectorXf(), Eigen::VectorXf(), Eigen::Array2f())
{
}

PlanningProblem::PlanningProblem(sim_env::WorldPtr world, sim_env::RobotPtr robot,
                                 sim_env::RobotVelocityControllerPtr controller,
                                 sim_env::ObjectPtr target_object,
                                 const Eigen::Vector3f& goal_position):
        world(world), robot(robot), robot_controller(controller), target_object(target_object),
        control_limits(Eigen::VectorXf(robot->getNumActiveDOFs()), Eigen::VectorXf(robot->getNumActiveDOFs()),
                       Eigen::Array2f()),
        goal_position(goal_position)
{

    planning_time_out = 60.0f;
    b_semi_dynamic = true;
    t_max = 8.0f;
    goal_region_radius = 0.05f;
    // create default workspace limits
    workspace_bounds.x_limits[0] = std::numeric_limits<float>::lowest();
    workspace_bounds.x_limits[1] = std::numeric_limits<float>::max();
    workspace_bounds.y_limits[0] = std::numeric_limits<float>::lowest();
    workspace_bounds.y_limits[1] = std::numeric_limits<float>::max();
    workspace_bounds.z_limits[0] = std::numeric_limits<float>::lowest();
    workspace_bounds.z_limits[1] = std::numeric_limits<float>::max();
    workspace_bounds.max_rotation_vel = 10.0f; // arbitrary maximum velocity in rad/s
    workspace_bounds.max_velocity = 30.0f; // arbitrary maximum velocity in m/s
    // Create default control limits
    control_limits.duration_limits[0] = 0.1; // minimal duration of holding the maximum velocity in s
    control_limits.duration_limits[1] = 1.0; // maximal duration of holding the maximum velocity in s
    Eigen::ArrayX2f acceleration_limits = robot->getDOFAccelerationLimits();
    Eigen::ArrayX2f velocity_limits = robot->getDOFVelocityLimits();
    assert(acceleration_limits.rows() == control_limits.acceleration_limits.rows());
    assert(velocity_limits.rows() == acceleration_limits.rows());
    for (unsigned int i = 0; i < acceleration_limits.rows(); ++i) {
        control_limits.acceleration_limits[i] = std::min(std::abs(acceleration_limits(i, 0)), acceleration_limits(i, 1));
        control_limits.velocity_limits[i] = std::min(std::abs(velocity_limits(i, 0)), velocity_limits(i, 1));
    }
    use_oracle = true;
    num_control_samples = 10;
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
    _planning_problem.use_oracle = false; // TODO remove
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
    using namespace std::placeholders;
    ::ompl::control::DirectedControlSamplerAllocator allocator_fn =
            std::bind(&OraclePushPlanner::allocateDirectedControlSampler, this, _1);
    _space_information->setDirectedControlSamplerAllocator(allocator_fn);
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
    std::vector<float> weights;
    prepareDistanceWeights(weights);
    _space_information->setStatePropagator(_state_propagator);
    _space_information->setup();
    _algorithm = std::make_shared<mps::planner::pushing::algorithm::SemiDynamicRRT>(_space_information, weights);
    _algorithm->setup();
    // TODO this is only for debug
    algorithm::SemiDynamicRRT::DebugDrawerPtr debug_drawer = std::make_shared<algorithm::SemiDynamicRRT::DebugDrawer>(_planning_problem.world->getViewer());
    _algorithm->setDebugDrawer(debug_drawer);
    // TODO we probably don't need to reconstruct everything all the time
    _is_initialized = true;
    return _is_initialized;
}

bool OraclePushPlanner::solve(PlanningSolution& solution) {
    // TODO run planning algorithm, extract solution and return it
    // start state
    auto* start_state = _state_space->allocState();
    _state_space->extractState(_planning_problem.world,
                               dynamic_cast<ompl::state::SimEnvWorldStateSpace::StateType *>(start_state));
    // goal
    ompl::state::goal::ObjectRelocationGoalPtr goal_region =
            std::make_shared<ompl::state::goal::ObjectRelocationGoal>(_space_information,
                                                                      _planning_problem.target_object->getName(),
                                                                      _planning_problem.goal_position,
                                                                      Eigen::Quaternionf(),
                                                                      _planning_problem.goal_region_radius,
                                                                      0.0f);
    // planning query
    algorithm::SemiDynamicRRT::PlanningQuery pq(goal_region, start_state, _planning_problem.planning_time_out);
    algorithm::SemiDynamicRRT::Path path(_space_information);
    bool success = _algorithm->plan(pq, path);
    _state_space->freeState(start_state);
    return success;
}

void OraclePushPlanner::dummyTest() {
    // TODO can put manual tests here
    ::ompl::base::StateSamplerPtr state_sampler = _space_information->allocStateSampler();
    ::ompl::control::ControlSamplerPtr control_sampler = _space_information->allocControlSampler();
    ::ompl::control::DirectedControlSamplerPtr directed_control_sampler = _space_information->allocDirectedControlSampler();
    ::ompl::base::State* state = _space_information->allocState();
    ::ompl::base::State* new_state = _space_information->allocState();
    ::ompl::control::Control* control = _space_information->allocControl();
    // CODE FOR TESTING NAIVE CONTROL SAMPLING
//    _distance_measure->setAll(false);
//    _distance_measure->setActive("robot", true);
//    state_sampler->sampleUniform(state);
//    state_sampler->sampleUniform(new_state);
//    auto* world_state = state->as<mps_state::SimEnvWorldState>();
//    auto* new_world_state = new_state->as<mps_state::SimEnvWorldState>();
//    util::logging::logDebug("Start state", "[mps::planner::pushing::OraclePushPlanner::dummyTest]");
//    _state_space->setToState(_planning_problem.world, world_state);
//    std::chrono::milliseconds sleeping_duration(1000);
//    std::this_thread::sleep_for(sleeping_duration);
//    util::logging::logDebug("Target state", "[mps::planner::pushing::OraclePushPlanner::dummyTest]");
//    _state_space->setToState(_planning_problem.world, new_world_state);
//    std::this_thread::sleep_for(sleeping_duration);
//    directed_control_sampler->sampleTo(control, state, new_state);
//    _state_propagator->propagate(state, control, new_state);
//    util::logging::logDebug("Resulting state of chosen action", "[mps::planner::pushing::OraclePushPlanner::dummyTest]");
//    _state_space->setToState(_planning_problem.world, new_world_state);
//    std::this_thread::sleep_for(sleeping_duration);

    // CODE FOR TESTING GOAL SAMPLING
//    ompl::state::goal::ObjectRelocationGoal or_goal(_space_information,
//                                                    _planning_problem.target_object->getName(),
//                                                    _planning_problem.goal_position,
//                                                    Eigen::Quaternionf(),
//                                                    _planning_problem.goal_region_radius,
//                                                    0.0f);
//    for (unsigned int i = 0; i < 10; ++i) {
//        or_goal.sampleGoal(state);
//        auto* world_state = state->as<mps_state::SimEnvWorldState>();
//        _state_space->setToState(_planning_problem.world, world_state);
//        std::chrono::milliseconds sleeping_duration(500);
//        std::this_thread::sleep_for(sleeping_duration);
//    }
    // CODE FOR GENERATING DUMMY DATA
    mps::planner::util::serialize::OracleDataDumper data_dumper;
    data_dumper.setFile("/home/joshua/test/oracle_training_data.cvs");
    for (unsigned int i = 0; i < 50; ++i) {
        state_sampler->sampleUniform(state);
        control_sampler->sample(control);
        auto* world_state = state->as<mps_state::SimEnvWorldState>();
        _state_space->setToState(_planning_problem.world, world_state);
        _planning_problem.world->getLogger()->logDebug("Sampled state");
        _state_propagator->propagate(state, control, new_state);
        _planning_problem.world->getLogger()->logDebug("Propagated state");
        world_state = new_state->as<mps_state::SimEnvWorldState>();
        _state_space->setToState(_planning_problem.world, world_state);
        data_dumper.saveData(state, new_state, control);
    }
    _space_information->freeState(state);
    _space_information->freeState(new_state);
    _space_information->freeControl(control);
}

void OraclePushPlanner::prepareDistanceWeights(std::vector<float> &weights) {
    weights.resize(_state_space->getNumObjects());
    for (unsigned int i = 0; i < weights.size(); ++i) {
        std::string name = _state_space->getObjectName(i);
        auto iter = _planning_problem.object_weights.find(name);
        if (iter != _planning_problem.object_weights.end()) {
            weights.at(i) = iter->second;
        } else {
            weights.at(i) = 1.0f;
        }
    }
}

::ompl::control::DirectedControlSamplerPtr OraclePushPlanner::allocateDirectedControlSampler(const ::ompl::control::SpaceInformation* si) {
    if (_planning_problem.use_oracle) {
        return std::make_shared<mps::planner::ompl::control::OracleControlSampler>(si);
    } else {
        return std::make_shared<mps::planner::ompl::control::NaiveControlSampler>(
               si, _planning_problem.num_control_samples);
    }
}

