#include <mps/planner/pushing/algorithm/Shortcut.h>

namespace logging = mps::planner::util::logging;
namespace ob = ::ompl::base;
namespace oc = ::ompl::control;
namespace mps_state = mps::planner::ompl::state;
namespace mps_control = mps::planner::ompl::control;
using namespace mps::planner::pushing::algorithm;
using namespace mps::planner::ompl::planning::essentials;
////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////// Shortcutter ///////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
Shortcutter::Shortcutter(::ompl::control::SpaceInformationPtr si)
    : _si(si)
    , _motion_cache(si)
{
    _state_propagator = std::dynamic_pointer_cast<mps_control::SimEnvStatePropagator>(si->getStatePropagator());
}

Shortcutter::~Shortcutter() = default;

void Shortcutter::setDebugDrawer(mps::planner::pushing::algorithm::DebugDrawerPtr debug_drawer)
{
    _debug_drawer = debug_drawer;
}

PathPtr Shortcutter::getNewPath()
{
    if (not _path_cache.empty()) {
        PathPtr ptr = _path_cache.top();
        _path_cache.pop();
        return ptr;
    }
    return std::make_shared<Path>(_si);
}

void Shortcutter::cachePath(PathPtr ptr, int clear_id)
{
    if (clear_id >= 0) {
        for (unsigned int id = clear_id; id < ptr->getNumMotions(); ++id) {
            _motion_cache.cacheMotion(ptr->getMotion(id));
        }
    }
    ptr->clear(); // make sure our path doesn't keep references to any motions
    _path_cache.push(ptr);
}

std::pair<bool, bool> Shortcutter::forwardPropagatePath(mps::planner::ompl::planning::essentials::PathPtr path,
    std::vector<mps::planner::ompl::planning::essentials::MotionPtr>& new_motions,
    ShortcutQuery& sq)
{
    bool goal_satisfied = false;
    bool extension_success = true;
    assert(path->getNumMotions() > 0);
    assert(not new_motions.empty());
    // start with prev_motion as last motion of path
    auto prev_motion = path->getMotion(path->getNumMotions() - 1);
    // propagate new_motions
    for (auto& new_motion : new_motions) {
        extension_success = _state_propagator->propagate(prev_motion->getState(),
            new_motion->getControl(),
            new_motion->getState());
        if (not extension_success) {
            return std::make_pair(false, false);
        }
        prev_motion = new_motion;
        path->append(new_motion);
        goal_satisfied = sq.goal_region->isSatisfied(new_motion->getState());
        if (goal_satisfied)
            return std::make_pair(true, true);
    }
    return std::make_pair(extension_success, goal_satisfied);
}

std::pair<bool, bool> Shortcutter::forwardPropagatePath(mps::planner::ompl::planning::essentials::PathPtr path,
    const std::vector<const ::ompl::control::Control*>& controls,
    ShortcutQuery& sq)
{
    std::vector<MotionPtr> motions;
    // first copy controls
    for (auto control : controls) {
        auto new_motion = _motion_cache.getNewMotion();
        _si->copyControl(new_motion->getControl(), control);
        motions.push_back(new_motion);
    }
    return forwardPropagatePath(path, motions, sq);
}

std::pair<bool, bool> Shortcutter::forwardPropagatePath(mps::planner::ompl::planning::essentials::PathPtr path,
    mps::planner::ompl::planning::essentials::PathPtr old_path,
    unsigned int old_path_continuation,
    ShortcutQuery& sq)
{
    bool extension_success = true;
    bool goal_satisfied = false;
    unsigned int current_idx = old_path_continuation;
    assert(path->getNumMotions() > 0);
    auto prev_motion = path->last();
    // as long as we have propagation success and there are actions in the old path left
    while (extension_success && current_idx < old_path->getNumMotions()) {
        auto new_motion = _motion_cache.getNewMotion();
        // copy next action from old path
        _si->copyControl(new_motion->getControl(), old_path->getMotion(current_idx)->getControl());
        // propagate from prev_motion
        extension_success = _state_propagator->propagate(prev_motion->getState(),
            new_motion->getControl(),
            new_motion->getState());
        if (extension_success) { // if successful propagation
            path->append(new_motion);
            prev_motion = new_motion;
            ++current_idx; // proceed in old path
            // check whether we reached a goal
            goal_satisfied = sq.goal_region->isSatisfied(new_motion->getState());
            if (goal_satisfied)
                return std::make_pair(true, true); // if yes we are done
        } else { // gonna abort
            _motion_cache.cacheMotion(new_motion);
        }
    }
    return std::make_pair(extension_success, goal_satisfied);
}

std::pair<bool, bool> Shortcutter::forwardPropagatePath(mps::planner::ompl::planning::essentials::PathPtr path,
    std::vector<mps::planner::ompl::planning::essentials::MotionPtr>& new_motions,
    mps::planner::ompl::planning::essentials::PathPtr old_path,
    unsigned int old_path_continuation,
    ShortcutQuery& sq)
{
    bool propagation_success = false;
    bool goal_reached = false;
    std::tie(propagation_success, goal_reached) = forwardPropagatePath(path, new_motions, sq);
    if (propagation_success && !goal_reached) {
        std::tie(propagation_success, goal_reached) = forwardPropagatePath(path, old_path, old_path_continuation, sq);
    }
    return std::make_pair(propagation_success, goal_reached);
}

void Shortcutter::showState(::ompl::base::State* state, const std::string& msg)
{
    std::stringstream ss;
    auto* world_state = dynamic_cast<mps_state::SimEnvWorldState*>(state);
    ss.str("");
    ss << msg;
    world_state->print(ss);
    logging::logDebug(ss.str(), "[Shortcutter::printState]");
#ifdef DEBUG_VISUALIZE
    if (_debug_drawer) {
        auto state_space = std::dynamic_pointer_cast<mps_state::SimEnvWorldStateSpace>(_si->getStateSpace());
        _debug_drawer->showState(world_state, state_space);
    }
#endif
}
////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////// NaiveShortcutter //////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
NaiveShortcutter::NaiveShortcutter(::ompl::control::SpaceInformationPtr si,
    mps::planner::pushing::oracle::RobotOraclePtr robot_oracle)
    : Shortcutter(si)
    , _robot_oracle(robot_oracle)
{
}

NaiveShortcutter::~NaiveShortcutter() = default;

void NaiveShortcutter::shortcut(mps::planner::ompl::planning::essentials::PathPtr iopath,
    ShortcutQuery& sq,
    float max_time)
{
    const std::string log_prefix("[NaiveShortcutter::shortcut]");
    logging::logDebug("Shortcutting path with naive shortcutter", log_prefix);
    std::vector<std::pair<unsigned int, unsigned int>> all_pairs;
    bool finished = iopath->getNumMotions() <= 2;
    if (finished)
        return;
    auto current_path = iopath->deepCopy(); // first copy the io path
    double current_path_cost = sq.cost_function->cost(current_path);
    double initial_cost = current_path_cost;
    auto world_state_space = std::dynamic_pointer_cast<mps_state::SimEnvWorldStateSpace>(_si->getStateSpace());
    unsigned int robot_id = world_state_space->getObjectIndex(sq.robot_name);
    _timer.startTimer(max_time); // now start the timer
    // run over all nodes in the path and try to shortcut
    while (!_timer.timeOutExceeded() && !finished && !sq.stopping_condition()) {
        // create all pairs
        createAllPairs(all_pairs, current_path->getNumMotions());
        auto pair_iter = all_pairs.begin();
        // try to shortcut
        while (!_timer.timeOutExceeded() && pair_iter != all_pairs.end() && !sq.stopping_condition()) {
            // gets indices to try shortcutting between
            auto start_id = pair_iter->first;
            auto end_id = pair_iter->second;
            // get motions
            // a motion is (action, state) where the action leads to the state
            auto first_wp = current_path->getMotion(start_id); // contains state from which to start shortcut
            auto snd_wp = current_path->getMotion(end_id); // contains state we would like to move to
            std::vector<MotionPtr> new_motions; // will contain new actions
            auto new_path = getNewPath(); // will contain new path
            // concat all waypoints up to start_id (incl)
            new_path->concat(current_path, start_id + 1);
            // do the actual shortcutting by computing robot actions that steer the robot
            computeRobotActions(new_motions, first_wp->getState(), snd_wp->getState(), robot_id);
            // test whether we still achieve the goal
            bool successful_path = false;
            std::tie(std::ignore, successful_path) = forwardPropagatePath(new_path, new_motions, current_path, end_id + 1, sq);
            double new_cost = sq.cost_function->cost(new_path);
            if (successful_path and (new_cost < current_path_cost)) {
                // replace path if new one is better
                logging::logDebug(boost::format("Found a shortcut. Old cost: %f, new cost %f") % current_path_cost % new_cost,
                    log_prefix);
                cachePath(current_path, start_id + 1);
                current_path = new_path;
                current_path_cost = new_cost;
                finished = current_path->getNumMotions() <= 2;
                break;
            } else {
                cachePath(new_path);
                _motion_cache.cacheMotions(new_motions);
            }
            ++pair_iter;
        }
        finished |= pair_iter == all_pairs.end(); // we are finished if we tried all pairs
    }
    // finally, we need to save the current path in iopath
    iopath->clear();
    for (unsigned int i = 0; i < current_path->getNumMotions(); ++i) {
        iopath->append(current_path->getMotion(i));
    }
    logging::logInfo(boost::format("Shortcut path from cost %f to %f.") % initial_cost % current_path_cost,
        log_prefix);
    sq.cost_before_shortcut = initial_cost;
    sq.cost_after_shortcut = current_path_cost;
}

std::string NaiveShortcutter::getName() const
{
    return "NaiveShortcutter";
}

void NaiveShortcutter::computeRobotActions(std::vector<mps::planner::ompl::planning::essentials::MotionPtr>& motions,
    ::ompl::base::State* start_state,
    ::ompl::base::State* end_state,
    unsigned int robot_id)
{
    auto sim_env_start_state = dynamic_cast<mps_state::SimEnvWorldState*>(start_state);
    auto sim_env_end_state = dynamic_cast<mps_state::SimEnvWorldState*>(end_state);
    auto current_robot_state = sim_env_start_state->getObjectState(robot_id);
    auto target_robot_state = sim_env_end_state->getObjectState(robot_id);
    std::vector<Eigen::VectorXf> control_params;
    _robot_oracle->steer(current_robot_state, target_robot_state, sim_env_start_state, control_params);
    for (auto& control_param : control_params) {
        auto new_motion = _motion_cache.getNewMotion();
        auto control = dynamic_cast<mps_control::RealValueParameterizedControl*>(new_motion->getControl());
        control->setParameters(control_param);
        motions.push_back(new_motion);
    }
}

void NaiveShortcutter::createAllPairs(std::vector<std::pair<unsigned int, unsigned int>>& all_pairs,
    unsigned int n) const
{
    all_pairs.clear();
    if (n <= 2)
        return; // nothing to do in this case
    all_pairs.reserve((n * n - 3 * n + 2) / 2); // number of pairs (skipping immediate neighbors)
    for (unsigned int i = 0; i < n - 2; ++i) {
        for (unsigned int j = i + 2; j < n; ++j) {
            all_pairs.push_back(std::make_pair(i, j));
        }
    }
    // Now shuffle the pairs
    std::random_device rd;
    std::mt19937 g(rd()); // TODO can we do this somehow with ompl's rng?
    std::shuffle(all_pairs.begin(), all_pairs.end(), g);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////// LocalShortcutter //////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

LocalShortcutter::LocalShortcutter(::ompl::control::SpaceInformationPtr si,
    mps::planner::pushing::oracle::RobotOraclePtr robot_oracle,
    const std::string& robot_name)
    : Shortcutter(si)
    , _robot_oracle(robot_oracle)
{
    auto world_state_space = std::dynamic_pointer_cast<mps_state::SimEnvWorldStateSpace>(_si->getStateSpace());
    _robot_id = world_state_space->getObjectIndex(robot_name);
}

LocalShortcutter::~LocalShortcutter() = default;

void LocalShortcutter::shortcut(mps::planner::ompl::planning::essentials::PathPtr iopath,
    ShortcutQuery& sq,
    float max_time)
{
    const std::string log_prefix("[LocalShortcutter::shortcut]");
    logging::logDebug("Shortcutting path with local shortcutter", log_prefix);
    bool finished = iopath->getNumMotions() <= 2;
    if (finished)
        return;
    auto current_path = iopath->deepCopy(); // first copy the io path
    auto trailing_path = getNewPath(); // will contain new trailing path
    double current_path_cost = sq.cost_function->cost(current_path);
    double initial_cost = current_path_cost;
    unsigned int stride = 2;
    _timer.startTimer(max_time); // now start the timer

    // repeat until we either tried everything, ran out of time, or the user interrupted
    while (!_timer.timeOutExceeded() && !finished && !sq.stopping_condition()) {
        // try to shortcut with current stride
        unsigned int start_id = 0;
        while (start_id + stride < current_path->getNumMotions() && !_timer.timeOutExceeded() && !sq.stopping_condition()) {
            // gets indices to try shortcutting between
            auto end_id = start_id + stride;
            // get motions
            // a motion is (action, state) where the action leads to the state
            auto first_wp = current_path->getMotion(start_id); // contains state from which to start shortcut
            auto snd_wp = current_path->getMotion(end_id); // contains state we would like to move to
#ifdef DEBUG_PRINTOUTS
            {
                showState(first_wp->getState(), "Start state of shortcut");
                showState(snd_wp->getState(), "Target state of shortcut");
            }
#endif
            std::vector<MotionPtr> new_motions; // will contain new actions
            trailing_path->clear();
            trailing_path->append(first_wp);
            // compute shortcut by appending actions to trailing_path that lead to snd_wp->getState()
            bool goal_path = false;
            bool valid_path = false;
            std::tie(valid_path, goal_path) = computeShortcut(trailing_path, snd_wp->getState(), sq);
            if (valid_path && !goal_path) { // if this worked and we haven't reached a goal yet, forward propagate rest of current_path
                std::tie(valid_path, goal_path) = forwardPropagatePath(trailing_path, current_path, end_id + 1, sq);
            }
            bool successful_path = valid_path && goal_path;
#ifdef DEBUG_PRINTOUTS
            {
                showState(trailing_path->last()->getState(), "Shortcut resulting final state");
            }
#endif
            double new_cost = sq.cost_function->cost(current_path, start_id) + sq.cost_function->cost(trailing_path);
            if (successful_path and (new_cost < current_path_cost)) {
                // concat all waypoints up to start_id (excl)
                auto new_path = getNewPath();
                new_path->concat(current_path, start_id);
                // concat all waypoints of trailing path (starts at start_id)
                new_path->concat(trailing_path);
                // replace path if new one is better
                logging::logDebug(boost::format("Found a shortcut. Old cost: %f, new cost %f") % current_path_cost % new_cost,
                    log_prefix);
                cachePath(current_path, start_id + 1);
                current_path = new_path;
                current_path_cost = new_cost;
                finished = current_path->getNumMotions() <= 2;
                // start_id = (unsigned int) std::max(0, (int)start_id + 1 - (int)stride);
                ++start_id;
            } else {
                _motion_cache.cacheMotions(new_motions);
                ++start_id;
            }
        }
        ++stride;
        finished = stride >= current_path->getNumMotions(); // we are finished if we tried all pairs
    }
    // finally, we need to save the current path in iopath
    iopath->clear();
    for (unsigned int i = 0; i < current_path->getNumMotions(); ++i) {
        iopath->append(current_path->getMotion(i));
    }
    logging::logInfo(boost::format("Shortcut path from cost %f to %f.") % initial_cost % current_path_cost,
        log_prefix);
    sq.cost_after_shortcut = current_path_cost;
    sq.cost_before_shortcut = initial_cost;
}

std::string LocalShortcutter::getName() const
{
    return "LocalShortcutter";
}

std::pair<bool, bool> LocalShortcutter::computeShortcut(mps::planner::ompl::planning::essentials::PathPtr prefix_path,
    ::ompl::base::State* end_state, ShortcutQuery& sq)
{
    auto first_wp = prefix_path->last();
    std::vector<MotionPtr> new_motions;
    // do the actual shortcutting by computing robot actions that steer the robot
    computeRobotActions(new_motions, first_wp->getState(), end_state);
    // test whether we still achieve the goal
    return forwardPropagatePath(prefix_path, new_motions, sq);
}

void LocalShortcutter::computeRobotActions(std::vector<mps::planner::ompl::planning::essentials::MotionPtr>& motions,
    ::ompl::base::State* start_state,
    ::ompl::base::State* end_state)
{
    auto sim_env_start_state = dynamic_cast<mps_state::SimEnvWorldState*>(start_state);
    auto sim_env_end_state = dynamic_cast<mps_state::SimEnvWorldState*>(end_state);
    auto current_robot_state = sim_env_start_state->getObjectState(_robot_id);
    auto target_robot_state = sim_env_end_state->getObjectState(_robot_id);
    std::vector<Eigen::VectorXf> control_params;
    _robot_oracle->steer(current_robot_state, target_robot_state, sim_env_start_state, control_params);
    for (auto& control_param : control_params) {
        auto new_motion = _motion_cache.getNewMotion();
        auto control = dynamic_cast<mps_control::RealValueParameterizedControl*>(new_motion->getControl());
        control->setParameters(control_param);
        motions.push_back(new_motion);
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////// LocalOracleShortcutter ////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
LocalOracleShortcutter::LocalOracleShortcutter(::ompl::control::SpaceInformationPtr si,
    mps::planner::pushing::oracle::RobotOraclePtr robot_oracle,
    mps::planner::pushing::oracle::PushingOraclePtr pushing_oracle,
    const std::string& robot_name)
    : LocalShortcutter(si, robot_oracle, robot_name)
{
    _oracle_sampler = std::make_shared<mps::planner::pushing::oracle::OracleControlSampler>(si, pushing_oracle, robot_oracle, robot_name);
}

LocalOracleShortcutter::~LocalOracleShortcutter() = default;

std::string LocalOracleShortcutter::getName() const
{
    return "LocalOracleShortcutter";
}

unsigned int LocalOracleShortcutter::selectObject(::ompl::base::State* start_state,
    ::ompl::base::State* end_state)
{
    auto start_world_state = dynamic_cast<mps_state::SimEnvWorldState*>(start_state);
    auto end_world_state = dynamic_cast<mps_state::SimEnvWorldState*>(end_state);
    auto state_space = std::dynamic_pointer_cast<mps_state::SimEnvWorldStateSpace>(_si->getStateSpace());
    unsigned int object_id = _robot_id;
    float max_distance = 0.0f;
    for (unsigned int i = 0; i < state_space->getNumObjects(); ++i) {
        if (i == _robot_id)
            continue; // not interested in the robot
        float dist = state_space->getSubspace(i)->distance(start_world_state->getObjectState(i),
            end_world_state->getObjectState(i));
        if (dist > max_distance) {
            max_distance = dist;
            object_id = i;
        }
    }
    return object_id;
}

std::pair<bool, bool> LocalOracleShortcutter::computeShortcut(
    mps::planner::ompl::planning::essentials::PathPtr prefix_path,
    ::ompl::base::State* end_state, ShortcutQuery& sq)
{
    static const std::string log_prefix("[LocalOracleShortcutter::computeShortcut]");
    std::vector<const ::ompl::control::Control*> controls; // stores controls computed by the oracle
    bool propagation_success = false; // stores whether propagations succeeed
    auto start = prefix_path->last();
    bool goal_reached = false;
    unsigned int object_id = selectObject(start->getState(), end_state);
    if (object_id != _robot_id) { // we plan to push object object_id
        // first sample a feasible state
        auto feasible_state_mtn = _motion_cache.getNewMotion();
        _si->copyState(feasible_state_mtn->getState(), start->getState());
        _oracle_sampler->samplePushingState(feasible_state_mtn->getState(),
            end_state,
            object_id);
        // steer robot to feasible state
        _oracle_sampler->steerRobot(controls, start->getState(), feasible_state_mtn->getState());
        _motion_cache.cacheMotion(feasible_state_mtn);
        if (controls.empty()) {
            logging::logErr("OracleControlSampler provided no controls at all", log_prefix);
            return std::make_pair(false, false);
        }
        // forward simulate this
        std::tie(propagation_success, goal_reached) = forwardPropagatePath(prefix_path, controls, sq);
        controls.clear();
        if (!propagation_success)
            return std::make_pair(false, false);
        if (goal_reached)
            return std::make_pair(true, true);
#ifdef DEBUG_PRINTOUTS
        {
            showState(prefix_path->last()->getState(), "Reached feasible state");
        }
#endif

        // next attempt to push
        _oracle_sampler->steerPush(controls, prefix_path->last()->getState(),
            end_state, object_id);
        // forward propagate these controls
        std::tie(propagation_success, goal_reached) = forwardPropagatePath(prefix_path, controls, sq);
        controls.clear();
        if (!propagation_success)
            return std::make_pair(false, false);
        if (goal_reached)
            return std::make_pair(true, true);
#ifdef DEBUG_PRINTOUTS
        {
            showState(prefix_path->last()->getState(), "Pushing outcome");
        }
#endif
    } // else only move the robot
    //  move the robot to the state where it has to be for the remaining path
    _oracle_sampler->steerRobot(controls, prefix_path->last()->getState(),
        end_state);
    std::tie(propagation_success, goal_reached) = forwardPropagatePath(prefix_path, controls, sq);
    controls.clear();
#ifdef DEBUG_PRINTOUTS
    {
        showState(prefix_path->last()->getState(), "Robot steered to its original position");
    }
#endif
    return std::make_pair(propagation_success, goal_reached);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////// OracleShortcutter /////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
OracleShortcutter::OracleShortcutter(::ompl::control::SpaceInformationPtr si,
    mps::planner::pushing::oracle::RobotOraclePtr robot_oracle,
    mps::planner::pushing::oracle::PushingOraclePtr pushing_oracle,
    const std::string& robot_name)
    : Shortcutter(si)
{
    _oracle_sampler = std::make_shared<mps::planner::pushing::oracle::OracleControlSampler>(si, pushing_oracle, robot_oracle, robot_name);
}

OracleShortcutter::~OracleShortcutter() = default;

void OracleShortcutter::shortcut(mps::planner::ompl::planning::essentials::PathPtr iopath,
    ShortcutQuery& sq,
    float max_time)
{
    const std::string log_prefix("[OracleShortcutter::shortcut]");
    logging::logDebug("Shortcutting path with oracle-based shortcutter", log_prefix);
    PairQueue all_pairs;
    PairMap pair_to_objects;
    bool finished = iopath->getNumMotions() <= 2;
    if (finished)
        return;
    auto current_path = iopath->deepCopy(); // first copy the io path
    double current_path_cost = sq.cost_function->cost(current_path);
    double initial_cost = current_path_cost;
    auto world_state_space = std::dynamic_pointer_cast<mps_state::SimEnvWorldStateSpace>(_si->getStateSpace());
    unsigned int robot_id = world_state_space->getObjectIndex(sq.robot_name);
    _timer.startTimer(max_time); // now start the timer
    // run over all nodes in the path and try to shortcut
    while (!_timer.timeOutExceeded() && !finished && !sq.stopping_condition()) {
        // create all pairs
        fillPairQueue(all_pairs, current_path->getNumMotions());
        pair_to_objects.clear();
        // try to shortcut
        while (!_timer.timeOutExceeded() && !all_pairs.empty() && !sq.stopping_condition()) {
            auto current_pair = all_pairs.front();
            all_pairs.pop_front();
            // gets indices to try shortcutting between
            auto start_id = current_pair.first;
            auto end_id = current_pair.second;
// print motions
#ifdef DEBUG_PRINTOUTS
            {
                auto first_mtn = current_path->getMotion(start_id);
                auto snd_mtn = current_path->getMotion(end_id);
                showState(first_mtn->getState(), "Shortcut attempt starts from state ");
                showState(snd_mtn->getState(), "And goes to state ");
            }
#endif
            // pick which object to push and also update pair data structures
            unsigned int object_id = selectObject(current_path, current_pair, pair_to_objects, all_pairs, robot_id);
            auto new_path = getNewPath(); // will contain new path
            // concat all waypoints up to start_id (incl)
            new_path->concat(current_path, start_id + 1);
            // do the actual shortcutting
            bool successful_path = computeShortcut(new_path, current_path, end_id,
                robot_id, object_id, sq);
            double new_cost = sq.cost_function->cost(new_path);
            if (successful_path and (new_cost < current_path_cost)) {
                // replace path if new one is better
                logging::logDebug(boost::format("Found a shortcut. Old cost: %f, new cost %f") % current_path_cost % new_cost,
                    log_prefix);
                cachePath(current_path, start_id + 1); // cache also motions from start_id + 1 onwards
                current_path = new_path;
                current_path_cost = new_cost;
                finished = current_path->getNumMotions() <= 2;
                break;
            } else {
                cachePath(new_path, start_id + 1); // cache also motions from start_id + 1 onwards
            }
        }
        finished |= all_pairs.empty();
    }
    // finally, we need to save the current path in iopath
    iopath->clear();
    for (unsigned int i = 0; i < current_path->getNumMotions(); ++i) {
        iopath->append(current_path->getMotion(i));
    }
    logging::logInfo(boost::format("Shortcut path from cost %f to %f.") % initial_cost % current_path_cost,
        log_prefix);
    sq.cost_before_shortcut = initial_cost;
    sq.cost_after_shortcut = current_path_cost;
}

std::string OracleShortcutter::getName() const
{
    return "OracleShortcutter";
}

void OracleShortcutter::fillPairQueue(PairQueue& all_pairs, unsigned int n) const
{
    all_pairs.clear();
    if (n <= 2)
        return; // nothing to do in this case
    for (unsigned int i = 0; i < n - 2; ++i) {
        for (unsigned int j = i + 2; j < n; ++j) {
            all_pairs.push_back(std::make_pair(i, j));
        }
    }
    // Now shuffle the pairs
    std::random_device rd;
    std::mt19937 g(rd()); // TODO can we do this somehow with ompl's rng?
    std::shuffle(all_pairs.begin(), all_pairs.end(), g);
}

unsigned int OracleShortcutter::selectObject(mps::planner::ompl::planning::essentials::PathPtr current_path,
    std::pair<unsigned int, unsigned int>& current_pair,
    PairMap& pair_to_objects, PairQueue& pair_queue,
    unsigned int robot_id) const
{
    auto map_iter = pair_to_objects.find(current_pair);
    ObjectIds object_ids;
    if (map_iter == pair_to_objects.end()) {
        // we have not tried this pair before
        auto first_mtn = current_path->getMotion(current_pair.first);
        auto snd_mtn = current_path->getMotion(current_pair.second);
        // compute which objects are different
        auto state_space = std::dynamic_pointer_cast<mps_state::SimEnvWorldStateSpace>(_si->getStateSpace());
        std::vector<std::pair<unsigned int, float>> distances;
        for (unsigned int i = 0; i < state_space->getNumObjects(); ++i) {
            if (i == robot_id)
                continue; // not interested in the robot
            auto first_state = dynamic_cast<mps_state::SimEnvWorldState*>(first_mtn->getState());
            auto snd_state = dynamic_cast<mps_state::SimEnvWorldState*>(snd_mtn->getState());
            float dist = state_space->getSubspace(i)->distance(first_state->getObjectState(i),
                snd_state->getObjectState(i));
            if (dist > 0.0f)
                distances.push_back(std::make_pair(i, dist));
        }
        // sort distances with objects that have largest distance last
        std::sort(distances.begin(), distances.end(),
            [](std::pair<unsigned int, float> a, std::pair<unsigned int, float> b) {
                return a.second < b.second;
            });
        // copy these indices to object_ids
        for (auto dist_idx_pair : distances) {
            object_ids.push_back(dist_idx_pair.first);
        }
    } else { // retrieve object ids from cache
        object_ids = map_iter->second;
    }
    if (object_ids.empty()) {
        // Move robot
        return robot_id;
    }
    // else pick the target id from the back of objects_ids
    auto target_id = object_ids.back();
    // remove it
    object_ids.pop_back();
    // update / place object_ids in cache
    pair_to_objects[current_pair] = object_ids;
    if (not object_ids.empty()) { // if there are objects left, we might want to try this pair again
        pair_queue.push_back(current_pair);
    }
    return target_id;
}

bool OracleShortcutter::computeShortcut(mps::planner::ompl::planning::essentials::PathPtr new_path,
    mps::planner::ompl::planning::essentials::PathPtr current_path,
    unsigned int destination_id, unsigned int robot_id,
    unsigned int object_id, ShortcutQuery& sq)
{
    static const std::string log_prefix("[mps::planner::pushing::algorithm::OracleShortcutter::computeShortcut]");
    std::vector<const ::ompl::control::Control*> controls; // stores controls computed by the oracle
    bool propagation_success = false; // stores whether propagations succeeed
    auto start = new_path->last();
    auto destination = current_path->getMotion(destination_id);
    bool goal_reached = false;
    if (object_id != robot_id) { // we plan to push object object_id
        // first sample a feasible state
        auto feasible_state_mtn = _motion_cache.getNewMotion();
        _si->copyState(feasible_state_mtn->getState(), start->getState());
        _oracle_sampler->samplePushingState(feasible_state_mtn->getState(),
            destination->getState(),
            object_id);
        // steer robot to feasible state
        _oracle_sampler->steerRobot(controls, start->getState(), feasible_state_mtn->getState());
        _motion_cache.cacheMotion(feasible_state_mtn);
        if (controls.empty()) {
            logging::logErr("OracleControlSampler provided no controls at all", log_prefix);
            return false;
        }
        // forward simulate this
        propagation_success = extendPath(new_path, controls, goal_reached, sq);
        controls.clear();
        if (!propagation_success)
            return false;
        if (goal_reached)
            return true;
#ifdef DEBUG_PRINTOUTS
        {
            showState(new_path->last()->getState(), "Reached feasible state");
        }
#endif

        // next attempt to push
        _oracle_sampler->steerPush(controls, new_path->last()->getState(),
            destination->getState(), object_id);
        // forward propagate these controls
        propagation_success = extendPath(new_path, controls, goal_reached, sq);
        controls.clear();
        if (!propagation_success)
            return false;
        if (goal_reached)
            return true;
#ifdef DEBUG_PRINTOUTS
        {
            showState(new_path->last()->getState(), "Pushing outcome");
        }
#endif
    } // else only move the robot
    //  move the robot to the state where it has to be for the remaining path
    _oracle_sampler->steerRobot(controls, new_path->last()->getState(),
        destination->getState());
    propagation_success = extendPath(new_path, controls, goal_reached, sq);
    controls.clear();
    if (!propagation_success)
        return false;
#ifdef DEBUG_PRINTOUTS
    {
        showState(new_path->last()->getState(), "Robot steered to its original position");
    }
#endif
    if (goal_reached)
        return true;
    // forward propagate the remaining path
    // first copy controls
    for (unsigned int i = destination_id + 1; i < current_path->getNumMotions(); ++i) {
        auto motion = current_path->getMotion(i);
        controls.push_back(motion->getControl());
    }
    propagation_success = extendPath(new_path, controls, goal_reached, sq);
    return propagation_success && goal_reached;
}

// propagates controls starting from  the end of the given path
bool OracleShortcutter::extendPath(PathPtr path,
    const std::vector<const ::ompl::control::Control*>& controls,
    bool& goal_reached, ShortcutQuery& sq)
{
    static const std::string log_prefix("[mps::planner::pushing::algorithm::OracleShortcutter::extendPath]");
    goal_reached = false;
    bool extension_success = false;
    auto prev_motion = path->last();
    for (auto const* control : controls) {
        MotionPtr new_motion = _motion_cache.getNewMotion();
        _si->copyControl(new_motion->getControl(), control);
        extension_success = _state_propagator->propagate(prev_motion->getState(),
            new_motion->getControl(),
            new_motion->getState());
        if (not extension_success) { // we failed, no shortcut
            logging::logDebug("Exending path failed", log_prefix);
            _motion_cache.cacheMotion(new_motion);
            return false;
        }
        // extension is successful, add this to the new path
        path->append(new_motion);
#ifdef DEBUG_PRINTOUTS
        {
            showState(new_motion->getState(), "new motion");
        }
#endif

        if (sq.goal_region->isSatisfied(new_motion->getState())) {
            // we reached a goal!
            goal_reached = true;
            return true;
        }
        // otherwise we just continue extending as long as we have controls
        prev_motion = new_motion;
    }
    return extension_success;
}