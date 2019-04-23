//
// Created by joshua on 8/21/17.
//

// stl
#include <chrono>
#include <functional>
#include <thread>
// mps
#include <mps/planner/ompl/control/NaiveControlSampler.h>
#include <mps/planner/ompl/control/RampVelocityControl.h>
#include <mps/planner/ompl/control/TimedWaypoints.h>
#include <mps/planner/ompl/state/goal/ObjectsRelocationGoal.h>
#include <mps/planner/pushing/Costs.h>
#include <mps/planner/pushing/OraclePushPlanner.h>
#include <mps/planner/pushing/algorithm/MultiExtendRRT.h>
#include <mps/planner/pushing/algorithm/SingleExtendRRT.h>
#include <mps/planner/pushing/oracle/HumanOracle.h>
#include <mps/planner/pushing/oracle/LearnedOracle.h>
#include <mps/planner/pushing/oracle/OracleControlSampler.h>
#include <mps/planner/pushing/oracle/QuasiStaticSE2Oracle.h>
#include <mps/planner/pushing/oracle/RampComputer.h>
#include <mps/planner/util/Logging.h>
#include <mps/planner/util/Playback.h>
#include <mps/planner/util/Serialize.h>
// ompl
#include <ompl/base/samplers/UniformValidStateSampler.h>

using namespace mps::planner::pushing;
namespace mps_state = mps::planner::ompl::state;
namespace mps_control = mps::planner::ompl::control;
namespace mps_algorithm = mps::planner::pushing::algorithm;
namespace mps_essentials = mps::planner::ompl::planning::essentials;
namespace mps_logging = mps::planner::util::logging;

PlanningProblem::PlanningProblem()
    : control_limits(Eigen::VectorXf(), Eigen::VectorXf(), Eigen::Array2f())
{
}

PlanningProblem::PlanningProblem(const PlanningProblem& other) = default;

PlanningProblem::PlanningProblem(sim_env::WorldPtr world, sim_env::RobotPtr robot,
    sim_env::RobotVelocityControllerPtr controller,
    const ompl::state::goal::RelocationGoalSpecification& goal)
    : PlanningProblem(world, robot, controller, std::vector<ompl::state::goal::RelocationGoalSpecification>(1, goal))
{
}

PlanningProblem::PlanningProblem(sim_env::WorldPtr world, sim_env::RobotPtr robot,
    sim_env::RobotVelocityControllerPtr controller,
    const std::vector<ompl::state::goal::RelocationGoalSpecification>& goals)
    : world(world)
    , robot(robot)
    , robot_controller(controller)
    , control_limits(Eigen::VectorXf(robot->getNumActiveDOFs()), Eigen::VectorXf(robot->getNumActiveDOFs()), Eigen::Array2f())
    , relocation_goals(goals)
{
    assert(relocation_goals.size() > 0);
    planning_time_out = 60.0f;
    b_semi_dynamic = true;
    t_max = 8.0f;
    goal_bias = 0.1f;
    robot_bias = 0.1f;
    target_bias = 0.1f;
    action_noise = 0.01f;
    state_noise = 0.001f;
    min_state_distance = 0.001;
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
    local_planner_type = LocalPlanner::Line;
    sdf_resolution = 0.002f;
    sdf_error_threshold = 0.1f;
    debug = false;
    num_control_samples = 10;
    shortcut_type = ShortcutType::LocalShortcut;
    max_shortcut_time = 5.0f;
    stopping_condition = []() { return false; };
    collision_policy.setStaticCollisions(true);
    collision_policy.setStaticCollisions(robot->getName(), false);
}

PlanningSolution::PlanningSolution()
    : path(nullptr)
    , pq(nullptr)
    , solved(false)
{
}

OraclePushPlanner::OraclePushPlanner()
{
    _is_initialized = false;
}

OraclePushPlanner::~OraclePushPlanner() = default;

bool OraclePushPlanner::setup(PlanningProblem& problem)
{
    // TODO this is ugly to have. In principle, I think this slice visualizer should maybe not be there or
    // TODO have it's copies of everything
    if (_rrt_debug_drawer) {
        // clearing the slice drawer here, allows it to delete slices before we reset the state space these slices belong to
        auto slice_drawer = _rrt_debug_drawer->getSliceDrawer();
        if (slice_drawer)
            slice_drawer->clear();
    }
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
    if (problem.oracle_type == PlanningProblem::OracleType::QuasiStaticSE2Oracle) {
        _control_space = std::make_shared<mps_control::TimedWaypointsControlSpace>(_state_space);
        _robot_controller = std::make_shared<sim_env::RobotPositionController>(_planning_problem.robot, _planning_problem.robot_controller);
    } else {
        _control_space = std::make_shared<mps_control::RampVelocityControlSpace>(_state_space,
            problem.control_limits,
            problem.control_subspaces);
        _robot_controller = _planning_problem.robot_controller;
    }
    using std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5;
    sim_env::Robot::ControlCallback callback = std::bind(&sim_env::RobotController::control,
        _robot_controller, _1, _2, _3, _4, _5);
    _planning_problem.robot->setController(callback);
    _space_information = std::make_shared<::ompl::control::SpaceInformation>(_state_space, _control_space);
    _space_information->setPropagationStepSize(1.0); // NOT USED
    _space_information->setMinMaxControlDuration(1, 1); // NOT USED
    _validity_checker = std::make_shared<mps::planner::ompl::state::SimEnvValidityChecker>(_space_information,
        _planning_problem.world);
    _validity_checker->collision_policy = _planning_problem.collision_policy;
    _space_information->setStateValidityChecker(_validity_checker);
    _state_propagator = std::make_shared<mps::planner::ompl::control::SimEnvStatePropagator>(_space_information,
        _planning_problem.world,
        _robot_controller,
        _planning_problem.b_semi_dynamic,
        _planning_problem.t_max);
    prepareDistanceWeights();
    _space_information->setStatePropagator(_state_propagator);
    _space_information->setup();
    createAlgorithm(); // sets _algorithm and _shortcutter
    _data_generator = nullptr;
    // TODO this is only for debug
    if (_planning_problem.debug) {
        std::vector<unsigned int> target_object_ids;
        for (auto& goal : _planning_problem.relocation_goals) {
            target_object_ids.emplace_back(_state_space->getObjectIndex(goal.object_name));
        }
        if (!_rrt_debug_drawer) {
            _rrt_debug_drawer = std::make_shared<algorithm::DebugDrawer>(_planning_problem.world->getViewer(),
                _state_space->getObjectIndex(_planning_problem.robot->getName()),
                target_object_ids);
        } else {
            _rrt_debug_drawer->setRobotId(_state_space->getObjectIndex(_planning_problem.robot->getName()));
            _rrt_debug_drawer->setTargetIds(target_object_ids);
        }
        _algorithm->setDebugDrawer(_rrt_debug_drawer);
        if (_shortcutter)
            _shortcutter->setDebugDrawer(_rrt_debug_drawer);
    }
    // TODO we probably don't need to reconstruct everything all the time
    _is_initialized = true;
    return _is_initialized;
}

bool OraclePushPlanner::solve(PlanningSolution& solution)
{
    if (!_is_initialized)
        throw std::logic_error("[OraclePushPlanner::solve] Planner not initialized. Can not plan.");
    if (_rrt_debug_drawer) {
        // TODO this is ugly to do like this
        auto slice_drawer = _rrt_debug_drawer->getSliceDrawer();
        if (slice_drawer)
            slice_drawer->setStateSpace(_state_space);
    }
    // start state
    auto* start_state = dynamic_cast<ompl::state::SimEnvWorldState*>(_state_space->allocState());
    _state_space->extractState(_planning_problem.world, start_state);
    // goal
    _goal_region = std::make_shared<ompl::state::goal::ObjectsRelocationGoal>(_space_information,
        _planning_problem.relocation_goals);
    // planning query
    solution.pq = _algorithm->createPlanningQuery(_goal_region, start_state,
        _planning_problem.robot->getName(),
        _planning_problem.planning_time_out);
    solution.pq->stopping_condition = _planning_problem.stopping_condition;
    solution.pq->weights = _distance_weights;
    std::setlocale(LC_NUMERIC, "en_US.UTF-8");
    if (solution.pq->parameters->hasParam("action_randomness")) {
        solution.pq->parameters->setParam("action_randomness", std::to_string(_planning_problem.action_noise));
    }
    if (solution.pq->parameters->hasParam("state_noise")) {
        solution.pq->parameters->setParam("state_noise", std::to_string(_planning_problem.state_noise));
    }
    if (solution.pq->parameters->hasParam("goal_bias")) {
        solution.pq->parameters->setParam("goal_bias", std::to_string(_planning_problem.goal_bias));
    }
    if (solution.pq->parameters->hasParam("target_bias")) {
        solution.pq->parameters->setParam("target_bias", std::to_string(_planning_problem.target_bias));
    }
    if (solution.pq->parameters->hasParam("robot_bias")) {
        solution.pq->parameters->setParam("robot_bias", std::to_string(_planning_problem.robot_bias));
    }
    if (solution.pq->parameters->hasParam("num_control_samples")) {
        solution.pq->parameters->setParam("num_control_samples", std::to_string(_planning_problem.num_control_samples));
    }
    // TODO set other parameters (tolerances)
    solution.path = nullptr;
    // before planning. let's save the state of the world
    auto world_state = _planning_problem.world->getWorldState();
    // PLAAAAAAANNN
    solution.solved = _algorithm->plan(solution.pq, solution.stats);
    if (solution.solved) {
        solution.path = solution.pq->path;
        solution.stats.reproducible = verifySolution(solution);
        if (!solution.stats.reproducible) {
            mps_logging::logWarn("Planner produced a non-reproducible solution", "[OraclePushPlanner::solve");
        }
    }
    // optionally shortcut query, TODO: shortcutting will likely fail if solution is not reproducible
    if (_planning_problem.shortcut_type != PlanningProblem::ShortcutType::NoShortcut && solution.solved) {
        auto cost_fn = std::make_shared<costs::ActionDurationCost>();
        algorithm::Shortcutter::ShortcutQuery sq(_goal_region, cost_fn, _planning_problem.robot->getName());
        sq.stopping_condition = _planning_problem.stopping_condition;
        assert(_shortcutter);
        _shortcutter->shortcut(solution.path, sq, _planning_problem.max_shortcut_time);
        solution.stats.cost_before_shortcut = sq.cost_before_shortcut;
        solution.stats.cost_after_shortcut = sq.cost_after_shortcut;
        solution.stats.reproducible_after_shortcut = verifySolution(solution);
    }
    // make sure the state of the world is the same as before
    _planning_problem.world->setWorldState(world_state);
    _state_space->freeState(start_state);
    return solution.solved;
}

void OraclePushPlanner::playback(const PlanningSolution& solution,
    const std::function<bool()>& interrupt_callback,
    bool force_synch)
{
    if (solution.solved) {
        clearVisualizations();
        mps::planner::util::playback::playPath(_planning_problem.world,
            _robot_controller,
            _state_space,
            solution.path,
            interrupt_callback,
            force_synch);
    }
}

bool OraclePushPlanner::execute(const PlanningSolution& solution,
    const ExecutionCallback& exec,
    const std::function<bool()>& interrupt_callback)
{
    bool exec_success = false;
    if (solution.solved) {
        ExecutionCallback callback;
        if (exec) {
            callback = exec;
        } else {
            callback = std::bind(mps::planner::util::playback::playMotion, _planning_problem.world, _robot_controller,
                _state_space, interrupt_callback, std::placeholders::_1, std::placeholders::_2);
        }
        _exec_monitor->setExecutionCallback(callback);
        exec_success = _exec_monitor->execute(solution.pq);
    }
    return exec_success;
}

bool OraclePushPlanner::saveSolution(const PlanningSolution& solution, const std::string& filename)
{
    if (!_is_initialized)
        throw std::logic_error("[OraclePushPlanner::saveSolution] Planner not initialized. Can not save solution.");
    static const std::string log_prefix("[OraclePushPlanner::saveSolution]");
    std::setlocale(LC_NUMERIC, "en_US.UTF-8");
    // first open the file
    std::fstream file_stream(filename.c_str(), std::fstream::out);
    if (!file_stream.is_open()) {
        mps_logging::logErr("Could not access file " + filename, log_prefix);
        return false;
    }
    // TODO save some header information, e.g. number of objects, semi-dynamic, DoFs of objects, DoFs action space
    // first save some information about the state and action space
    // first line contains state space information (#object, has_velocities, #dofs_obj1, #dofs_obj2, ...)
    file_stream << _state_space->getNumObjects() << ",";
    file_stream << _state_space->hasVelocities() << ",";
    for (unsigned int i = 0; i < _state_space->getNumObjects(); ++i) {
        file_stream << _state_space->getSubspace(i)->getDimension();
        if (i < _state_space->getNumObjects() - 1)
            file_stream << ",";
    }
    file_stream << "\n";
    // second line contains action space information
    auto serializable_control_space = std::dynamic_pointer_cast<mps_control::SerializableControlSpace>(_control_space);
    assert(serializable_control_space);
    serializable_control_space->serializeSpaceInformation(file_stream);
    // third line contains stats
    solution.stats.printCVS(file_stream);
    // run over path and store it
    file_stream << solution.path->getNumMotions() << "\n";
    for (unsigned int wp = 0; wp < solution.path->getNumMotions(); ++wp) {
        auto motion = solution.path->getConstMotion(wp);
        auto state = dynamic_cast<const mps_state::SimEnvWorldState*>(motion->getConstState());
        state->serializeInNumbers(file_stream);
        file_stream << "\n";
        auto control = dynamic_cast<const mps::planner::util::serialize::RealValueSerializable*>(motion->getConstControl());
        control->serializeInNumbers(file_stream);
        file_stream << "\n";
    }
    file_stream.close();
    return true;
}

bool OraclePushPlanner::loadSolution(PlanningSolution& solution, const std::string& filename)
{
    if (!_is_initialized)
        throw std::logic_error("[OraclePushPlanner::loadSolution] Planner not initialized. Can not load solution.");
    static const std::string log_prefix("[OraclePushPlanner::loadSolution]");
    std::setlocale(LC_NUMERIC, "en_US.UTF-8");

    // Try opening the file first
    std::fstream file_stream(filename.c_str(), std::fstream::in);
    if (!file_stream.is_open()) {
        mps_logging::logErr("Could not access file " + filename, log_prefix);
        return false;
    }
    // Read in header and compare if it is compatible to the set state and control space
    mps_logging::logDebug("Reading in header", log_prefix);
    std::string line;
    std::getline(file_stream, line);
    std::vector<std::string> comma_seperated_values;
    mps::planner::util::serialize::splitString(line, comma_seperated_values);
    if (comma_seperated_values.size() < 3) {
        mps_logging::logErr("Could not load solution. Invalid header!", log_prefix);
    }
    if (std::stoul(comma_seperated_values[0]) != _state_space->getNumObjects()) {
        mps_logging::logErr("Could not load solution. Invalid number of objects in state space.", log_prefix);
        return false;
    }
    // read in whether it has velocities or not
    bool has_velocities = std::stoul(comma_seperated_values[1]);
    if ((has_velocities && !_state_space->hasVelocities()) || (!has_velocities && _state_space->hasVelocities())) {
        mps_logging::logErr("Could not load solution. Semi-dynamic flag is incompatible.", log_prefix);
        return false;
    }
    if (comma_seperated_values.size() != _state_space->getNumObjects() + 2) {
        mps_logging::logErr("Could not load solution. Incorrect number of object DoFs in header", log_prefix);
        return false;
    }
    // check remaining object spaces for sanity
    for (unsigned int i = 0; i < _state_space->getNumObjects(); ++i) {
        unsigned int dof = std::stoul(comma_seperated_values[i + 2]);
        if (dof != _state_space->getSubspace(i)->getDimension()) {
            mps_logging::logErr(boost::format("Could not load solution. Dimension for object %i is incompatible.") % i,
                log_prefix);
            return false;
        }
    }
    auto serialize_control_space = std::dynamic_pointer_cast<mps_control::SerializableControlSpace>(_control_space);
    assert(serialize_control_space);
    // check action space dimension (new line)
    if (!serialize_control_space->deserializeSpaceInformation(file_stream)) {
        mps_logging::logErr("Could not load solution. Action space is incompatible", log_prefix);
        return false;
    }
    //*************** Read in stats **************//
    mps_logging::logDebug("Reading in stats", log_prefix);
    solution.stats.readCVS(file_stream);
    //*************** Read in path **************//
    mps_logging::logDebug("Reading in path", log_prefix);
    std::getline(file_stream, line);
    unsigned int num_motions = std::stoul(line);
    if (solution.path) {
        solution.path->clear();
    } else {
        solution.path = std::make_shared<mps_essentials::Path>(_space_information);
    }
    mps_logging::logDebug(boost::format("Reading in %i number of motions") % num_motions, log_prefix);
    for (unsigned int i = 0; i < num_motions; ++i) {
        auto new_motion = std::make_shared<mps_essentials::Motion>(_space_information);
        std::getline(file_stream, line);
        {
            std::stringstream line_stream(line);
            auto state = dynamic_cast<mps_state::SimEnvWorldState*>(new_motion->getState());
            state->deserializeFromNumbers(line_stream);
        }
        std::getline(file_stream, line);
        {
            std::stringstream line_stream(line);
            auto control = dynamic_cast<mps::planner::util::serialize::RealValueSerializable*>(new_motion->getControl());
            control->deserializeFromNumbers(line_stream);
        }
        solution.path->append(new_motion);
    }
    solution.solved = true;
    return true;
}

void OraclePushPlanner::setSliceDrawer(algorithm::SliceDrawerInterfacePtr slice_drawer)
{
    if (_rrt_debug_drawer)
        _rrt_debug_drawer->setSliceDrawer(slice_drawer);
}

void OraclePushPlanner::renderSDF(float resolution)
{
    // if (_eb_computer) {
    //     _eb_debug_drawer = _eb_computer->getDebugDrawer();
    //     _eb_computer->renderSDF(resolution);
    // }
}

void OraclePushPlanner::clearVisualizations()
{
    if (_rrt_debug_drawer) {
        _rrt_debug_drawer->clear();
        auto slice_drawer = _rrt_debug_drawer->getSliceDrawer();
        if (slice_drawer)
            slice_drawer->clear();
    }
    // if (_eb_debug_drawer) {
    //     _eb_debug_drawer->clear();
    // }
}

void OraclePushPlanner::generateData(const std::string& file_name,
    unsigned int num_samples,
    const std::string& annotation,
    bool deterministic)
{
    // TODO what about other algorithms?
    if (!_data_generator) {
        _data_generator = std::make_shared<oracle::DataGenerator>(_space_information, _state_propagator,
            _planning_problem.world, _planning_problem.robot->getName(),
            _planning_problem.relocation_goals.at(0).object_name);
    }
    _data_generator->generateData(file_name, num_samples, annotation, deterministic);
}

void OraclePushPlanner::evaluateOracle(mps::planner::ompl::state::goal::RelocationGoalSpecification goal,
    const std::string& file_name,
    unsigned int num_samples,
    const std::string& annotation)
{
    // TODO what about other algorithms?
    auto oracle_rrt = std::dynamic_pointer_cast<algorithm::OracleRearrangementRRT>(_algorithm);
    auto oracle_sampler = oracle_rrt->getOracleSampler();
    if (!_data_generator) {
        _data_generator = std::make_shared<oracle::DataGenerator>(_space_information, _state_propagator,
            _planning_problem.world, _planning_problem.robot->getName(),
            _planning_problem.relocation_goals.at(0).object_name);
    }
    _data_generator->evaluateOracle(goal, oracle_sampler, _state_space, file_name, num_samples, annotation);
}

void OraclePushPlanner::dummyTest()
{
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

bool OraclePushPlanner::verifySolution(PlanningSolution& solution)
{
    // TODO update to be able to work with PushMotion (teleports)
    // first make sure we actually have a path
    if (!solution.path)
        return false;
    if (solution.path->getNumMotions() <= 2)
        return false;
    // now we verify that it is actually valid and leading to a goal
    bool extension_success = true;
    auto my_motion = std::make_shared<mps_essentials::Motion>(_space_information);
    _state_space->copyState(my_motion->getState(), solution.path->getMotion(0)->getState());
    // first propagate new_motions
    for (unsigned int m = 1; m < solution.path->getNumMotions(); ++m) {
        auto current_motion = solution.path->getMotion(m);
        _control_space->copyControl(my_motion->getControl(), current_motion->getConstControl());
        extension_success = _state_propagator->propagate(my_motion->getState(),
            my_motion->getControl(),
            my_motion->getState());
        if (not extension_success) {
            return false;
        }
    }
    // if we reached this point, the path is valid. Let's check whether it also led to a goal
    return _goal_region->isSatisfied(my_motion->getState());
}

mps::planner::ompl::planning::essentials::PathPtr OraclePushPlanner::testOracle(const ompl::state::goal::RelocationGoalSpecification& goal, bool approach) const
{
    ///////////////////////////////////// First make sure everything is set up properly /////////////////////////////
    const static std::string log_prefix("[mps::planner::pushing::OraclePushPlanner]");
    if (not _is_initialized) {
        auto logger = sim_env::DefaultLogger::getInstance();
        logger->logErr("Could not test oracle. Planner is not initialized. Call setup(...) first.", log_prefix);
        return nullptr;
    }
    auto logger = _planning_problem.world->getLogger();
    int target_id = _state_space->getObjectIndex(goal.object_name);
    int robot_id = _state_space->getObjectIndex(_planning_problem.robot->getName());
    assert(robot_id >= 0); // the robot is a sim_env object, so it should always be a valid id here
    if (target_id < 0) {
        logger->logErr("Could not test oracle - invalid target name " + goal.object_name, log_prefix);
        return nullptr;
    }
    // get oracle sampled
    oracle::OracleControlSamplerPtr oracle_sampler = nullptr;
    auto oracle_rrt = std::dynamic_pointer_cast<algorithm::OracleRearrangementRRT>(_algorithm);
    if (oracle_rrt) {
        oracle_sampler = oracle_rrt->getOracleSampler();
    } else {
        auto mextrrt = std::dynamic_pointer_cast<algorithm::MultiExtendRRT>(_algorithm);
        if (!mextrrt) {
            logger->logErr("Could not test oracle. The selected algorithm does not use an oracle.", log_prefix);
            return nullptr;
        }
        oracle_sampler = mextrrt->getOracleSampler();
    }
    ///////////////////////////////////// Now we can ask the oracle /////////////////////////////
    auto* target_state = dynamic_cast<mps::planner::ompl::state::SimEnvWorldState*>(_state_space->allocState());
    auto start_motion = std::make_shared<ompl::planning::essentials::Motion>(_space_information);
    _state_space->extractState(_planning_problem.world,
        dynamic_cast<mps::planner::ompl::state::SimEnvWorldState*>(start_motion->getState()));
    _state_space->copyState(target_state, start_motion->getState());
    auto* object_state = target_state->getObjectState(target_id);
    Eigen::VectorXf target_config(3);
    target_config.head(3) = goal.goal_position.head(3);
    object_state->setConfiguration(target_config);
    std::vector<::ompl::control::Control*> oracle_controls;
    if (target_id == robot_id) {
        oracle_sampler->steerRobot(oracle_controls, start_motion->getState(), target_state);
    } else {
        // check whether we want to move to a pushing state or
        if (approach) {
            auto pushing_state = dynamic_cast<mps::planner::ompl::state::SimEnvWorldState*>(_state_space->allocState());
            _state_space->copyState(pushing_state, start_motion->getState());
            oracle_sampler->samplePushingState(pushing_state, target_state, target_id);
            oracle_sampler->steerRobot(oracle_controls, start_motion->getState(), pushing_state);
            _state_space->freeState(pushing_state);
        } else {
            auto* push_control = _control_space->allocControl();
            oracle_sampler->queryPolicy(push_control, dynamic_cast<ompl::state::SimEnvWorldState*>(start_motion->getState()), target_state, target_id);
            oracle_controls.push_back(push_control);
        }
    }
    // Done, cleanup
    _state_space->freeState(target_state);
    // Create Path
    auto path = std::make_shared<ompl::planning::essentials::Path>(_space_information);
    _control_space->copyControl(start_motion->getControl(), oracle_controls.at(0));
    _control_space->freeControl(oracle_controls.at(0));
    path->append(start_motion);
    for (size_t i = 1; i < oracle_controls.size(); ++i) {
        auto new_motion = std::make_shared<ompl::planning::essentials::Motion>(_space_information);
        _control_space->copyControl(new_motion->getControl(), oracle_controls.at(i));
        new_motion->setParent(path->getMotion(i - 1));
        _control_space->freeControl(oracle_controls.at(i));
        path->append(new_motion);
    }
    return path;
}

mps::planner::ompl::state::SimEnvWorldStateSpacePtr OraclePushPlanner::getStateSpace()
{
    return _state_space;
}

::ompl::control::ControlSpacePtr OraclePushPlanner::getControlSpace()
{
    return _control_space;
}

mps::planner::ompl::state::SimEnvValidityCheckerPtr OraclePushPlanner::getValidityChecker()
{
    return _validity_checker;
}

mps::planner::ompl::control::SimEnvStatePropagatorPtr OraclePushPlanner::getStatePropagator()
{
    return _state_propagator;
}

void OraclePushPlanner::prepareDistanceWeights()
{
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

void OraclePushPlanner::createAlgorithm()
{
    static const std::string log_prefix("[mps::planner::pushing::OraclePushPlanner::createAlgorithm]");
    if (_planning_problem.algorithm_type == PlanningProblem::AlgorithmType::Naive) {
        _algorithm = std::make_shared<algorithm::NaiveRearrangementRRT>(_space_information);
        util::logging::logDebug("Using naive algorithm, i.e. no oracle at all", log_prefix);
        if (_planning_problem.shortcut_type != PlanningProblem::ShortcutType::NoShortcut) {
            util::logging::logWarn("Selected naive planner in combination with shortcutting."
                                   "Shortcutting not supported in this setting",
                log_prefix);
        }
    } else {
        // all other algorithms need an oracle
        auto robot_state_space = _state_space->getObjectStateSpace(_planning_problem.robot->getName());
        auto robot_configuration_space = robot_state_space->getConfigurationSpace();
        unsigned int robot_id = _state_space->getObjectIndex(_planning_problem.robot->getName());
        oracle::PushingOraclePtr pushing_oracle;
        oracle::RobotOraclePtr robot_oracle;
        switch (_planning_problem.local_planner_type) {
        case PlanningProblem::LocalPlanner::PotentialField: {
            // if (!_eb_computer) {
            // _eb_computer = std::make_shared<oracle::ElasticBandRampComputer>();
            // }
            // _eb_computer->init(_planning_problem.world,
            //     _planning_problem.robot,
            //     _state_space,
            //     _control_space,
            //     _planning_problem.workspace_bounds,
            //     _planning_problem.sdf_resolution,
            //     _planning_problem.sdf_error_threshold);
            // robot_oracle = _eb_computer;
            util::logging::logErr("PotentialField no longer supported.", log_prefix);
            // robot_oracle = nullptr;
            // break;
        }
        case PlanningProblem::LocalPlanner::Line: {
            if (_planning_problem.oracle_type == PlanningProblem::OracleType::QuasiStaticSE2Oracle) {
                auto twp_control_space = std::dynamic_pointer_cast<ompl::control::TimedWaypointsControlSpace>(_control_space);
                assert(twp_control_space);
                // TODO minimal velocity should be treated differently for cartesian and rotational
                robot_oracle = std::make_shared<ompl::control::TimedWaypointsRobotOracle>(robot_state_space,
                    robot_id, twp_control_space, _planning_problem.control_limits.velocity_limits.minCoeff());
            } else {
                auto ramp_control_space = std::dynamic_pointer_cast<ompl::control::RampVelocityControlSpace>(_control_space);
                assert(ramp_control_space);
                auto ramp_computer = std::make_shared<oracle::RampComputer>(robot_configuration_space, ramp_control_space, robot_id);
                robot_oracle = ramp_computer;
            }
            break;
        }
        }
        // create object data
        std::vector<sim_env::ObjectPtr> objects;
        for (unsigned int i = 0; i < _state_space->getNumObjects(); ++i) {
            // TODO Change state space to be able to return non-const object pointers?
            objects.push_back(_planning_problem.world->getObject(_state_space->getObjectName(i)));
        }
        switch (_planning_problem.oracle_type) {
        case PlanningProblem::OracleType::Human: {
            pushing_oracle = std::make_shared<oracle::HumanOracle>(robot_oracle, robot_id, objects);
            util::logging::logInfo("Using human made oracle!", log_prefix);
            break;
        }
        case PlanningProblem::OracleType::Learned: {
            pushing_oracle = std::make_shared<oracle::LearnedPipeOracle>(objects, robot_id);
            util::logging::logInfo("Using learned oracle!", log_prefix);
            break;
        }
        case PlanningProblem::OracleType::QuasiStaticSE2Oracle: {
            pushing_oracle = std::make_shared<oracle::QuasiStaticSE2Oracle>(objects, robot_id);
            util::logging::logInfo("Using quasistatic oracle!", log_prefix);
            break;
        }
        }
        switch (_planning_problem.algorithm_type) {
        case PlanningProblem::AlgorithmType::OracleRRT: {
            _algorithm = std::make_shared<algorithm::OracleRearrangementRRT>(_space_information,
                pushing_oracle,
                robot_oracle,
                _planning_problem.robot->getName());
            util::logging::logInfo("Using OracleRRT", log_prefix);
            _exec_monitor = std::make_shared<algorithm::ExecutionMonitor>(_algorithm);
            break;
        }
        case PlanningProblem::AlgorithmType::SliceOracleRRT: {
            _algorithm = std::make_shared<algorithm::SliceBasedOracleRRT>(_space_information,
                pushing_oracle,
                robot_oracle,
                _planning_problem.robot->getName());
            _exec_monitor = std::make_shared<algorithm::ExecutionMonitor>(_algorithm);
            util::logging::logInfo("Using SliceOracleRRT", log_prefix);
            break;
        }
        case PlanningProblem::AlgorithmType::HybridActionRRT: {
            _algorithm = std::make_shared<algorithm::HybridActionRRT>(_space_information,
                pushing_oracle,
                robot_oracle, _planning_problem.robot->getName());
            util::logging::logInfo("Using HybridActionRRT", log_prefix);
            _exec_monitor = std::make_shared<algorithm::ExecutionMonitor>(_algorithm);
            break;
        }
        case PlanningProblem::AlgorithmType::GreedyMultiExtendRRT: {
            auto merrtplanner = std::make_shared<algorithm::GreedyMultiExtendRRT>(_space_information,
                pushing_oracle,
                robot_oracle,
                _planning_problem.robot->getName());
            _algorithm = merrtplanner;
            _exec_monitor = std::make_shared<algorithm::MERRTExecutionMonitor>(merrtplanner);
            break;
        }
        default: {
            util::logging::logErr("Invalid algorithm configuration encountered.", log_prefix);
            throw std::runtime_error("Invalid algorithm configuration encountered");
        }
        }
        if (_planning_problem.shortcut_type != PlanningProblem::ShortcutType::NoShortcut) {
            createShortcutAlgorithm(pushing_oracle, robot_oracle);
        }
    }
}

void OraclePushPlanner::createShortcutAlgorithm(oracle::PushingOraclePtr pushing_oracle,
    oracle::RobotOraclePtr robot_oracle)
{
    std::string log_prefix("[OraclePushPlanner::createShortcutAlgorithm]");
    switch (_planning_problem.shortcut_type) {
    case PlanningProblem::ShortcutType::NaiveShortcut: {
        util::logging::logInfo("Setting up naive shortcutter", log_prefix);
        _shortcutter = std::make_shared<algorithm::NaiveShortcutter>(_space_information, robot_oracle);
        break;
    }
    case PlanningProblem::ShortcutType::LocalShortcut: {
        util::logging::logInfo("Setting up local shortcutter", log_prefix);
        _shortcutter = std::make_shared<algorithm::LocalShortcutter>(_space_information, robot_oracle,
            _planning_problem.robot->getName());
        break;
    }
    case PlanningProblem::ShortcutType::LocalOracleShortcut: {
        util::logging::logInfo("Setting up local oracle shortcutter", log_prefix);
        _shortcutter = std::make_shared<algorithm::LocalOracleShortcutter>(_space_information, robot_oracle, pushing_oracle,
            _planning_problem.robot->getName());
        break;
    }
    case PlanningProblem::ShortcutType::OracleShortcut: {
        util::logging::logInfo("Setting up oracle shortcutter", log_prefix);
        _shortcutter = std::make_shared<algorithm::OracleShortcutter>(_space_information, robot_oracle, pushing_oracle,
            _planning_problem.robot->getName());
        break;
    }
    case PlanningProblem::ShortcutType::NoShortcut: {
        util::logging::logInfo("No shortcutting selected.", log_prefix);
        break;
    }
    default: {
        util::logging::logErr("Invalid shortcut configuration encountered.", log_prefix);
        throw std::runtime_error("Invalid shortcut configuration encountered");
    }
    }
}