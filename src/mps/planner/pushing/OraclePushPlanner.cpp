//
// Created by joshua on 8/21/17.
//

#include <ompl/base/samplers/UniformValidStateSampler.h>
#include <mps/planner/pushing/OraclePushPlanner.h>
#include <mps/planner/util/Serialize.h>
#include <mps/planner/ompl/state/goal/ObjectRelocationGoal.h>
#include <mps/planner/pushing/oracle/OracleControlSampler.h>
#include <mps/planner/ompl/control/NaiveControlSampler.h>
#include <mps/planner/util/Logging.h>
#include <mps/planner/util/Playback.h>
#include <mps/planner/pushing/oracle/HumanOracle.h>
#include <mps/planner/pushing/oracle/LearnedOracle.h>
#include <mps/planner/pushing/oracle/RampComputer.h>
#include <thread>
#include <chrono>

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
    goal_bias = 0.1f;
    robot_bias = 0.1f;
    target_bias = 0.1f;
    num_slice_neighbors = 8;
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
    oracle_type = OracleType::Human;
    algorithm_type = AlgorithmType::Naive;
    debug = false;
    num_control_samples = 10;
    stopping_condition = [](){return false;};
    collision_policy.setStaticCollisions(true);
    collision_policy.setStaticCollisions(robot->getName(), false);
}

PlanningSolution::PlanningSolution() : path(nullptr), solved(false) {

}

OraclePushPlanner::OraclePushPlanner() {
    _is_initialized = false;
}

OraclePushPlanner::~OraclePushPlanner() = default;

bool OraclePushPlanner::setup(PlanningProblem& problem) {
    static const std::string log_prefix("[mps::planner::pushing::OraclePushPlanner::setup]");
    // first delete any previous instances
    _space_information.reset();
    _state_space.reset();
    _control_space.reset();
    _validity_checker.reset();
    _state_propagator.reset();
    _planning_problem = problem;
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
                                                                                 _planning_problem.collision_policy,
                                                                                 _planning_problem.b_semi_dynamic,
                                                                                 _planning_problem.t_max);
    prepareDistanceWeights();
    _space_information->setStatePropagator(_state_propagator);
    _space_information->setup();
    _algorithm = createAlgorithm(_planning_problem);
    // TODO this is only for debug
    if (_planning_problem.debug) {
        if (!_debug_drawer) {
            _debug_drawer = std::make_shared<algorithm::RearrangementRRT::DebugDrawer>(_planning_problem.world->getViewer(),
                                                                                       _state_space->getObjectIndex(_planning_problem.robot->getName()),
                                                                                       _state_space->getObjectIndex(_planning_problem.target_object->getName()));
        }
        _algorithm->setDebugDrawer(_debug_drawer);
    }
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
    algorithm::RearrangementRRT::PlanningQuery pq(goal_region,
                                                  start_state,
                                                  _planning_problem.planning_time_out,
                                                  _planning_problem.target_object->getName(),
                                                  _planning_problem.robot->getName());
    pq.stopping_condition = _planning_problem.stopping_condition;
    pq.weights = _distance_weights;
    pq.num_slice_neighbors = _planning_problem.num_slice_neighbors;
    pq.goal_bias = _planning_problem.goal_bias;
    pq.robot_bias = _planning_problem.robot_bias;
    pq.target_bias = _planning_problem.target_bias;
    solution.path = std::make_shared<mps::planner::ompl::planning::essentials::Path>(_space_information);
    solution.solved = _algorithm->plan(pq, solution.path, solution.stats);
    _state_space->freeState(start_state);
    return solution.solved;
}

void OraclePushPlanner::playback(const PlanningSolution& solution) {
    if (solution.solved) {
        clearVisualizations();
        mps::planner::util::playback::playPath(_planning_problem.world,
                                               _planning_problem.robot_controller,
                                               _state_space,
                                               solution.path);
    }
}

void OraclePushPlanner::clearVisualizations() {
    if (_debug_drawer) {
        _debug_drawer->clear();
    }
}

void OraclePushPlanner::generateData(const std::string& file_name,
                                     unsigned int num_samples,
                                     const std::string& header) {
    std::shared_ptr<::ompl::base::UniformValidStateSampler> state_sampler =
            std::make_shared<::ompl::base::UniformValidStateSampler>(_space_information.get());
    state_sampler->setNrAttempts(100);
    ::ompl::control::ControlSamplerPtr control_sampler = _space_information->allocControlSampler();
    ::ompl::base::State* state = _space_information->allocState();
    ::ompl::base::State* new_state = _space_information->allocState();
    ::ompl::control::Control* control = _space_information->allocControl();
    mps::planner::util::serialize::OracleDataDumper data_dumper;
    data_dumper.setFile(file_name);
    data_dumper.openFile();
    data_dumper.writeHeader(header);
    unsigned int i = 0;
    while (i < num_samples) {
        bool has_state = state_sampler->sample(state);
        if (not has_state) {
            _planning_problem.world->getLogger()->logWarn("Failed to sample a valid state, we might end up in a infinite loop here!");
            continue;
        }
        control_sampler->sample(control);
        _planning_problem.world->getLogger()->logDebug("Sampled state");
        bool success = _state_propagator->propagate(state, control, new_state);
        _planning_problem.world->getLogger()->logDebug("Propagated state");
        if (not success) {
            _planning_problem.world->getLogger()->logDebug("State propagation failed, skipping");
            continue;
        }
        double state_distance = _space_information->distance(state, new_state);
        if (state_distance <= 0.0000001) {
            _planning_problem.world->getLogger()->logWarn("Resulting state is equal to start state");
        }
        data_dumper.saveData(state, new_state, control);
        ++i;
    }
    data_dumper.closeFile();
    _space_information->freeState(state);
    _space_information->freeState(new_state);
    _space_information->freeControl(control);
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

void OraclePushPlanner::prepareDistanceWeights() {
    _distance_weights.resize(_state_space->getNumObjects());
    for (unsigned int i = 0; i < _distance_weights.size(); ++i) {
        std::string name = _state_space->getObjectName(i);
        auto iter = _planning_problem.object_weights.find(name);
        if (iter != _planning_problem.object_weights.end()) {
            _distance_weights.at(i) = iter->second;
        } else {
            _distance_weights.at(i) = 1.0f;
        }
    }
}

mps::planner::pushing::algorithm::RearrangementRRTPtr OraclePushPlanner::createAlgorithm(const PlanningProblem& pp) const {
    static const std::string log_prefix("[mps::planner::pushing::OraclePushPlanner::createAlgorithm]");
    mps::planner::pushing::algorithm::RearrangementRRTPtr algo;
    if (pp.algorithm_type == PlanningProblem::AlgorithmType::Naive)
    {
        algo = std::make_shared<algorithm::NaiveRearrangementRRT>(_space_information,
                                                                  _planning_problem.num_control_samples);
        util::logging::logDebug("Using naive algorithm, i.e. no oracle at all", log_prefix);
    } else {
        // both other algorithms need an oracle
        auto robot_state_space = _state_space->getObjectStateSpace(_planning_problem.robot->getName());
        auto robot_configuration_space = robot_state_space->getConfigurationSpace();
        oracle::PushingOraclePtr pushing_oracle;
        auto ramp_computer =
                std::make_shared<oracle::RampComputer>(robot_configuration_space, _control_space);
        oracle::RobotOraclePtr robot_oracle = ramp_computer;
        switch (_planning_problem.oracle_type) {
            case PlanningProblem::OracleType::Human:
            {
                pushing_oracle = std::make_shared<oracle::HumanOracle>(ramp_computer);
                util::logging::logDebug("Using human made oracle!", log_prefix);
                break;
            }
            case PlanningProblem::OracleType::Learned:
            {
                // mps::planner::pushing::oracle::PushingOraclePtr pushing_oracle = std::make_shared<oracle::LearnedPipeOracle>();
                // TODO fix me
                pushing_oracle = nullptr;
                util::logging::logDebug("Using learned oracle!", log_prefix);
                break;
            }
        }
        if (pp.algorithm_type == PlanningProblem::AlgorithmType::OracleRRT) {
            algo = std::make_shared<algorithm::OracleRearrangementRRT>(_space_information,
                                                                       pushing_oracle,
                                                                       robot_oracle,
                                                                       _planning_problem.robot->getName());
        } else {
            algo = std::make_shared<algorithm::SliceBasedOracleRRT>(_space_information,
                                                                    pushing_oracle,
                                                                    robot_oracle,
                                                                    _planning_problem.robot->getName());
        }
    }
    return algo;
}

