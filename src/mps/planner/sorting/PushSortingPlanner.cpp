//
// Created by joshua on 7/02/18.
//

#include <mps/planner/sorting/PushSortingPlanner.h>
#include <mps/planner/util/Serialize.h>
#include <mps/planner/ompl/control/NaiveControlSampler.h>
#include <mps/planner/util/Logging.h>
#include <mps/planner/util/Playback.h>
#include <thread>
#include <chrono>

using namespace mps::planner::sorting;
namespace mps_state = mps::planner::ompl::state;
namespace mps_control = mps::planner::ompl::control;
namespace mps_essentials = mps::planner::ompl::planning::essentials;
namespace mps_logging = mps::planner::util::logging;

PlanningProblem::PlanningProblem() {
    // set default values
    planning_time_out = 60.0f;
    t_max = 8.0f;
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
    // other default values
    value_fn_type = ValueFunctionType::Entropy;
    algorithm_type = AlgorithmType::DeterministicMCTS;
    debug = false;
    num_control_samples = 10;
    stopping_condition = [](){return false;};
    collision_policy.setStaticCollisions(false);
}

PlanningProblem::PlanningProblem(const PlanningProblem& other) = default;

bool PlanningProblem::init_robot() 
{
    if (!world || !robot_controller) return false;
    robot = robot_controller->getRobot();
    Eigen::ArrayX2f acceleration_limits = robot->getDOFAccelerationLimits();
    Eigen::ArrayX2f velocity_limits = robot->getDOFVelocityLimits();
    control_limits.acceleration_limits.resize(robot->getNumActiveDOFs());
    control_limits.velocity_limits.resize(robot->getNumActiveDOFs());
    for (unsigned int i = 0; i < acceleration_limits.rows(); ++i) {
        control_limits.acceleration_limits[i] = std::min(std::abs(acceleration_limits(i, 0)), acceleration_limits(i, 1));
        control_limits.velocity_limits[i] = std::min(std::abs(velocity_limits(i, 0)), velocity_limits(i, 1));
    }
    return true;
}

PlanningSolution::PlanningSolution() : path(nullptr), solved(false) {
}

PushSortingPlanner::PushSortingPlanner() {
    _is_initialized = false;
}

PushSortingPlanner::~PushSortingPlanner() = default;

bool PushSortingPlanner::setup(PlanningProblem& problem) {
    static const std::string log_prefix("[mps::planner::sorting::PushSortingPlanner::setup]");
    // first delete any previous instances
    _space_information.reset();
    _state_space.reset();
    _control_space.reset();
    _validity_checker.reset();
    _state_propagator.reset();
    _planning_problem = problem;
    // now create new instances
    // This creates a SimEnvWorldStateSpace from the given world
    _state_space = std::make_shared<mps_state::SimEnvWorldStateSpace>(_planning_problem.world,
                                                                      _planning_problem.workspace_bounds,
                                                                      true,
                                                                      mps_state::SimEnvWorldStateSpace::WeightMap());
    // This creates a RampVelocityControlSpace. This is an action space consisting of
    // velocitiies in a plane (x,y,theta) that follow a ramp shaped velocity profile.
    // In addition to its x,y,theta velocities, an action is characeterized by its duration.
    _control_space =
            std::make_shared<mps_control::RampVelocityControlSpace>(_state_space,
                                                                    problem.control_limits,
                                                                    problem.control_subspaces);
    // Space information is a class from OMPL that serves as a container for all information
    // related to search spaces (state and actions)
    _space_information =
            std::make_shared<::ompl::control::SpaceInformation>(_state_space, _control_space);
    _space_information->setPropagationStepSize(1.0); // NOT USED
    _space_information->setMinMaxControlDuration(1, 1); // NOT USED
    // a validity checker allows us to check whether a state is valid or not
    _validity_checker =
            std::make_shared<mps::planner::ompl::state::SimEnvValidityChecker>(_space_information,
                                                                               _planning_problem.world);
    // we can specify what kind of contact is allowed by setting a collision policy
    _validity_checker->collision_policy = _planning_problem.collision_policy;
    _space_information->setStateValidityChecker(_validity_checker);
    // a state propagator wraps the physics model and predicts what the outcome of an action
    // is applied to a given state
    _state_propagator =
            std::make_shared<mps::planner::ompl::control::SimEnvStatePropagator>(_space_information,
                                                                                 _planning_problem.world,
                                                                                 _planning_problem.robot_controller,
                                                                                 true,
                                                                                 _planning_problem.t_max);
    _space_information->setStatePropagator(_state_propagator);
    _space_information->setup();
    createAlgorithm(); // sets _algorithm
    _is_initialized = true;
    return _is_initialized;
}

bool PushSortingPlanner::solve(PlanningSolution& solution) {
    if (!_is_initialized) throw std::logic_error("[PushSortingPlanner::solve] Planner not initialized. Can not plan.");
    // extract the start state from the world
    auto* start_state = _state_space->allocState(); // this creates a new state object
    _state_space->extractState(_planning_problem.world,
                               dynamic_cast<ompl::state::SimEnvWorldStateSpace::StateType *>(start_state));
    if (_state_space->getNumObjects() <= 2) return true; // TODO: or should we return false?
    // extract groups
    std::vector<unsigned int> groups(_state_space->getNumObjects(), 0);
    for (auto& element : _planning_problem.sorting_groups) {
        auto idx = _state_space->getObjectIndex(element.first);
        groups[idx] = element.second;
    }
    // create a planning query object. this serves as input to the actual planning algorithm
    algorithm::MCTSBase::PlanningQuery pq(start_state,
                                          _planning_problem.planning_time_out,
                                          groups,
                                          _planning_problem.robot->getName());
    pq.stopping_condition = _planning_problem.stopping_condition;
    pq.num_control_samples = _planning_problem.num_control_samples;
    // ccreate a path object, in which we can store the solution the planner finds
    solution.path = std::make_shared<mps::planner::ompl::planning::essentials::Path>(_space_information);
    // before we plan, let's save the state of the world
    auto world_state = _planning_problem.world->getWorldState();
    // Note that this state is of a different type, do not use this state for planning.
    // it's just for restoring the world's state
    // Anyhow, here you can call you planning algorithm
    solution.solved = _algorithm->plan(pq, solution.path, solution.stats);
    // optionally shortcut query, TODO: shortcutting will likely fail if solution is not reproducible
    // now restore the state of the world
    _planning_problem.world->setWorldState(world_state);
    _state_space->freeState(start_state); // delete the start state
    return solution.solved;
}

void PushSortingPlanner::playback(const PlanningSolution& solution,
                                 const std::function<bool()>& interrupt_callback,
                                 bool force_synch) {
    if (solution.solved) {
        mps::planner::util::playback::playPath(_planning_problem.world,
                                               _planning_problem.robot_controller,
                                               _state_space,
                                               solution.path,
                                               interrupt_callback,
                                               force_synch);
    }
}

void PushSortingPlanner::createAlgorithm() {
    // In this function, we create the actual algorithm
    static const std::string log_prefix("[mps::planner::pushing::PushSortingPlanner::createAlgorithm]");
    switch (_planning_problem.algorithm_type) {
        case PlanningProblem::AlgorithmType::DeterministicMCTS:
        {
            // at first you will work on this algorithm
            _algorithm = std::make_shared<algorithm::DeterministicMCTS>(_space_information);
            util::logging::logDebug("Using deterministic MCTS algorithm", log_prefix);
            break;
        }
        case PlanningProblem::AlgorithmType ::NonDeterministicMCTS:
        {
            // maybe we can also look into non-deterministic MCTS, so just in case
            _algorithm = std::make_shared<algorithm::NonDeterministicMCTS>(_space_information);
            util::logging::logInfo("Using non-deterministic MCTS algorithm", log_prefix);
            break;
        }
        default:
        {
            util::logging::logErr("Invalid algorithm configuration encountered.", log_prefix);
            throw std::runtime_error("Invalid algorithm configuration encountered");
        }
    }
}
