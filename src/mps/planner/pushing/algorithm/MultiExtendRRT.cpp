#include <algorithm>
#include <mps/planner/pushing/algorithm/MultiExtendRRT.h>
#include <ompl/base/ScopedState.h>
#include <ompl/datastructures/NearestNeighborsGNAT.h>

namespace logging = mps::planner::util::logging;
namespace ob = ::ompl::base;
namespace oc = ::ompl::control;
namespace mps_state = mps::planner::ompl::state;
namespace mps_control = mps::planner::ompl::control;
namespace mps_oracle = mps::planner::pushing::oracle;
using namespace mps::planner::pushing::algorithm;
using namespace mps::planner::ompl::planning::essentials;

// util funtions
bool madeProgress(const mps_state::SimEnvWorldState* before, const mps_state::SimEnvWorldState* after,
    const mps_state::SimEnvWorldState* target_state, unsigned int t, mps_state::SimEnvWorldStateSpacePtr state_space)
{
    float dist_before = state_space->objectStateDistance(before, target_state, t);
    float dist_after = state_space->objectStateDistance(after, target_state, t);
    return dist_after < dist_before;
}

bool madeProgress(PushMotionPtr x_b, PushMotionPtr x_a, SliceConstPtr target_slice, unsigned int t,
    mps_state::SimEnvWorldStateSpacePtr state_space)
{
    auto object_state_space = state_space->getSubspace(t);
    auto before_state = dynamic_cast<mps_state::SimEnvWorldState*>(x_b->getState());
    auto after_state = dynamic_cast<mps_state::SimEnvWorldState*>(x_a->getState());
    auto target_state = dynamic_cast<mps_state::SimEnvWorldState*>(target_slice->repr->getState());
    return madeProgress(before_state, after_state, target_state, t, state_space);
}

PushMotion::PushMotion(oc::SpaceInformationPtr si)
    : Motion(si)
    , _target_id(0)
    , _is_teleport_transit(false)
{
}

PushMotion::PushMotion(const PushMotion& other)
    : Motion(other._weak_si.lock())
{
    _weak_si = other._weak_si;
    auto si = _weak_si.lock();
    if (!si) {
        std::logic_error("[mps::planner::ompl::planning::essentials::Motion::PushMotion(const PushMotion&)]"
                         " Could not access space information. The other motion is invalid.");
    }
    si->copyState(_state, other._state);
    si->copyControl(_control, other._control);
    _parent = other._parent;
    _target_id = other._target_id;
    _is_teleport_transit = other._is_teleport_transit;
}

PushMotion::~PushMotion() = default;

PushMotion& PushMotion::operator=(const PushMotion& other)
{
    _weak_si = other._weak_si;
    auto si = _weak_si.lock();
    if (!si) {
        std::logic_error("[mps::planner::ompl::planning::essentials::PushMotion::operator=]"
                         " Could not access space information. The other motion is invalid.");
    }
    si->copyState(_state, other._state);
    si->copyControl(_control, other._control);
    _parent = other._parent;
    _target_id = other._target_id;
    _is_teleport_transit = other._is_teleport_transit;
    return *this;
}

void PushMotion::setTargetId(unsigned int id)
{
    _target_id = id;
}

unsigned int PushMotion::getTargetId() const
{
    return _target_id;
}

void PushMotion::setTeleportTransit(bool btransit)
{
    _is_teleport_transit = btransit;
}

bool PushMotion::isTeleportTransit() const
{
    return _is_teleport_transit;
}

void PushMotion::reset()
{
    Motion::reset();
    _target_id = 0;
    _is_teleport_transit = false;
}

MotionPtr PushMotion::deepCopy() const
{
    auto si = _weak_si.lock();
    if (!si) {
        throw std::logic_error("Could not copy Motion. Space Information does no longer exist.");
    }
    auto new_motion = std::make_shared<PushMotion>(si);
    si->copyState(new_motion->getState(), _state);
    si->copyControl(new_motion->getControl(), _control);
    new_motion->setTargetId(_target_id);
    new_motion->setTeleportTransit(_is_teleport_transit);
    return new_motion;
}

/**************************************** MultiExtendRRT *******************************************/
MultiExtendRRT::MultiExtendRRT(oc::SpaceInformationPtr si, oracle::PushingOraclePtr pushing_oracle,
    oracle::RobotOraclePtr robot_oracle, const std::string& robot_name)
    : RearrangementPlanner(si)
    , _robot_state_dist_fn(std::make_shared<RobotStateDistanceFn>(_state_space))
    , _slice_distance_fn(std::make_shared<ObjectArrangementDistanceFn>(_state_space))
    , _pushing_oracle(pushing_oracle)
    , _oracle_sampler(std::make_shared<mps_oracle::OracleControlSampler>(si, pushing_oracle, robot_oracle, robot_name))
    , _min_slice_distance(0.0)
    , _goal_bias(0.1)
    , _motion_cache(si)
    , _slice_cache(_robot_state_dist_fn)
    , _log_prefix("[mps::planner::pushing::algorithm::MultiExtendRRT::")
{
    _rng = mps::planner::util::random::getDefaultRandomGenerator();
    _state_propagator = std::dynamic_pointer_cast<mps_control::SimEnvStatePropagator>(si->getStatePropagator());
    assert(_state_propagator);
    _slices_nn = std::make_shared<::ompl::NearestNeighborsGNAT<SlicePtr>>();
    _slices_nn->setDistanceFunction(std::bind(&ObjectArrangementDistanceFn::distance,
        std::ref(_slice_distance_fn),
        std::placeholders::_1,
        std::placeholders::_2));
    _state_sampler = _si->allocValidStateSampler();
}

MultiExtendRRT::~MultiExtendRRT() = default;

bool MultiExtendRRT::plan(PlanningQueryPtr pq, PlanningStatistics& stats)
{
    static const std::string log_prefix(_log_prefix + "plan]");
    // setup
    PlanningBlackboard blackboard(pq);
    setup(pq, blackboard);
    logging::logDebug("Starting to plan", log_prefix);
    PushMotionPtr current_motion = _motion_cache.getNewMotion();
    PushMotionPtr sample_motion = _motion_cache.getNewMotion();
    PushMotionPtr final_motion = nullptr;
    SlicePtr current_slice = nullptr;
    // set the state to be the start state
    _si->copyState(current_motion->getState(), pq->start_state);
    // set the control to be null
    _si->nullControl(current_motion->getControl());
    // Initialize the tree
    addToTree(current_motion, nullptr, blackboard);
    final_motion = current_motion;
    current_slice = getSlice(current_motion, blackboard);
    // check whether the problem is already solved
    bool solved = pq->goal_region->isSatisfied(current_motion->getState());
    // debug printouts
    std::stringstream ss;
    pq->goal_region->print(ss);
    logging::logDebug("Planning towards goal " + ss.str(), log_prefix);
    logging::logDebug("Entering main loop", log_prefix);
    _timer->startTimer(pq->time_out);
    // Do the actual planning
    while (not _timer->timeOutExceeded() and not pq->stopping_condition() && !solved) {
        blackboard.stats.num_iterations++;
        // sample a new target arrangement
        sample(sample_motion, blackboard);
#ifdef DEBUG_PRINTOUTS
        printState("Sampled arrangement is ", sample_motion->getState());
#endif
        // select the slice to expand
        select(sample_motion, current_slice, blackboard);
#ifdef DEBUG_PRINTOUTS
        printState("Selected tree state: ", current_motion->getState()); // TODO remove
#endif
        // Extend the tree
        solved = extend(current_slice, sample_motion, final_motion, blackboard);
#ifdef DEBUG_PRINTOUTS
        if (final_motion) {
            printState("Tree extended to ", final_motion->getState()); // TODO remove
        } else {
            logging::logDebug("Failed to extend tree", log_prefix);
        }
#endif
    }

    blackboard.stats.runtime = _timer->stopTimer();
    blackboard.stats.success = solved;
    ss.str("");
    blackboard.stats.print(ss);
    logging::logDebug("Main loop finished, stats:\n" + ss.str(), log_prefix);
    // create the path if we found a solution
    if (solved) {
        logging::logInfo("Found a solution", log_prefix);
        pq->path = std::make_shared<mps::planner::ompl::planning::essentials::Path>(_si);
        pq->path->initBacktrackMotion(final_motion);
    }
    // clean up
    _slices_nn->clear();
    logging::logInfo("Planning finished", log_prefix);
    stats = blackboard.stats;
    return solved;
}

RearrangementPlanner::PlanningQueryPtr MultiExtendRRT::createPlanningQuery(
    mps_state::goal::ObjectsRelocationGoalPtr goal_region,
    mps_state::SimEnvWorldState* start_state,
    const std::string& robot_name,
    float timeout)
{
    auto pq = RearrangementPlanner::createPlanningQuery(goal_region, start_state, robot_name, timeout);
    using std::placeholders::_1;
    // declare goal bias
    std::function<void(float)> gb_setter = std::bind(&MultiExtendRRT::setGoalBias, this, _1);
    std::function<float()> gb_getter = std::bind(&MultiExtendRRT::getGoalBias, this);
    pq->parameters->declareParam<float>("goal_bias", gb_setter, gb_getter);
    pq->parameters->setParam("goal_bias", "0.2");
    // declare pushing state epsilon
    std::function<void(float)> ps_setter = std::bind(&MultiExtendRRT::setPushingStateEps, this, _1);
    std::function<float()> ps_getter = std::bind(&MultiExtendRRT::getPushingStateEps, this);
    pq->parameters->declareParam<float>("pushing_state_eps", ps_setter, ps_getter);
    pq->parameters->setParam("pushing_state_eps", "0.00001");
    return pq;
}

void MultiExtendRRT::setGoalBias(float goal_bias)
{
    _goal_bias = goal_bias;
}

float MultiExtendRRT::getGoalBias() const
{
    return _goal_bias;
}

void MultiExtendRRT::setPushingStateEps(float eps)
{
    _pushing_state_eps = eps;
}

float MultiExtendRRT::getPushingStateEps() const
{
    return _pushing_state_eps;
}

bool MultiExtendRRT::isGoalPath(PathConstPtr path, const mps_state::SimEnvWorldState* start, PlanningQueryPtr pq, PathPtr updated_path)
{
    if (!_si->isValid(start))
        return false;
    auto motion = std::make_shared<PushMotion>(_si);
    auto prev_motion = motion;
    _si->copyState(motion->getState(), start);
    _si->nullControl(motion->getControl());
    unsigned int robot_id = _state_space->getSubspaceIndex(pq->robot_name);
    bool valid_path = true;
    // run over path and forward simulate motions
    for (unsigned int i = 0; i < path->getNumMotions(); ++i) {
        auto m = path->getConstMotion(i);
        auto pm = std::dynamic_pointer_cast<const PushMotion>(m);
        motion->setTargetId(pm->getTargetId());
        if (pm and pm->isTeleportTransit()) {
            _si->nullControl(motion->getControl());
            motion->setTeleportTransit(true);
            // copy previous state
            _state_space->copyState(motion->getState(), prev_motion->getState());
            // copy robot state (teleportation)
            _state_space->copySubState(motion->getState(), m->getConstState(), robot_id);
            valid_path = _si->isValid(motion->getState());
        } else {
            // propagate action
            _si->copyControl(motion->getControl(), m->getConstControl());
            valid_path = _state_propagator->propagate(prev_motion->getState(), motion->getControl(), motion->getState());
        }
        if (!valid_path)
            break;
        if (updated_path) {
            updated_path->append(motion);
            prev_motion = motion;
            motion = std::make_shared<PushMotion>(_si);
        }
    }
    bool goal_reached = pq->goal_region->isSatisfied(prev_motion->getState());
    return goal_reached && valid_path;
}

void MultiExtendRRT::sample(const PushMotionPtr& sample, PlanningBlackboard& pb)
{
    static const std::string log_prefix("mps::planner::pushing::algorithm::MultiExtendRRT::sample]");
    // sample random state with goal biasing
    if (_rng->uniform01() < _goal_bias && pb.pq->goal_region->canSample()) {
        logging::logDebug("Sampling a goal arrangement", log_prefix);
        pb.pq->goal_region->sampleGoal(sample->getState());
    } else {
        logging::logDebug("Sampling a random arrangement", log_prefix);
        _state_sampler->sample(sample->getState());
    }
    pb.stats.num_samples++;
}

void MultiExtendRRT::select(const PushMotionPtr& sample, SlicePtr& selected_slice, PlanningBlackboard& pb) const
{
    static const std::string log_prefix("[mps::planner::pushing::algorithm::MultiExtendRRT]");
    logging::logDebug("Selecting slice to extend to given sample", log_prefix);
    // pick the slice that is closest to the sample
    selected_slice = getSlice(sample, pb);
#ifdef DEBUG_PRINTOUTS
    printState("Rerpresentative of nearest slice is ", selected_slice->repr->getState());
#endif
}

void MultiExtendRRT::setup(PlanningQueryPtr pq, PlanningBlackboard& blackboard)
{
    static const std::string log_prefix(_log_prefix + "setup]");
    setupBlackboard(blackboard);
    if (_debug_drawer) {
        _debug_drawer->clear();
    }
    // TODO should update this using state space and distance weights
    _min_slice_distance = (_state_space->getNumObjects() - 1) * 0.001;
    _slice_distance_fn->distance_measure.setWeights(pq->weights);
    _slice_distance_fn->setRobotId(blackboard.robot_id);
    _robot_state_dist_fn->setRobotId(blackboard.robot_id);
    _slices_nn->clear();
    _pushing_oracle->timer = _timer;
    logging::logInfo("MultiExtendRRT setup for the following query:\n " + blackboard.pq->toString(), log_prefix);
}

SlicePtr MultiExtendRRT::addToTree(PushMotionPtr new_motion, PushMotionPtr root, PlanningBlackboard& pb)
{
    std::vector<MotionPtr> path;
    MotionPtr motion = new_motion;
    while (motion != nullptr) {
        path.push_back(motion);
        motion = motion->getParent();
    }
    path.back()->setParent(root);
    SlicePtr closest_slice;
    for (auto iter = path.rbegin(); iter != path.rend(); iter++) {
        // _tree->add(new_motion);
        if (_debug_drawer) {
            _debug_drawer->addNewMotion(*iter);
        }
        closest_slice = getSlice(*iter, pb);
        float slice_distance = distanceToSlice(*iter, closest_slice);
        if (slice_distance > _min_slice_distance) { // we discovered a new slice!
            auto new_slice = _slice_cache.getNewSlice(*iter);
            _slices_nn->add(new_slice);
            if (_debug_drawer) {
                _debug_drawer->addNewSlice(new_slice);
            }
        } else { // the new motion/state is in the same slice
            closest_slice->addSample(*iter);
        }
    }
    return closest_slice;
}

SlicePtr MultiExtendRRT::getSlice(MotionPtr motion, PlanningBlackboard& pb) const
{
    if (_slices_nn->size() == 0) {
        return nullptr;
    }
    auto query_slice = _slice_cache.getNewSlice(motion);
    auto nearest = _slices_nn->nearest(query_slice);
    ++pb.stats.num_nearest_neighbor_queries;
    _slice_cache.cacheSlice(query_slice);
    return nearest;
}

float MultiExtendRRT::distanceToSlice(ompl::planning::essentials::MotionPtr motion, SlicePtr slice) const
{
    if (!slice) {
        return std::numeric_limits<float>::max();
    }
    auto query_slice = _slice_cache.getNewSlice(motion);
    float distance = (float)_slice_distance_fn->distance(query_slice, slice);
    _slice_cache.cacheSlice(query_slice);
    return distance;
}

bool MultiExtendRRT::samplePush(const ompl::state::SimEnvWorldState* start_state,
    const ompl::state::SimEnvWorldState* target_state,
    PushMotionPtr approach, PushMotionPtr push,
    unsigned int t,
    bool sample_pushing_state)
{
    _state_space->copyState(approach->getState(), start_state);
    if (sample_pushing_state) {
        // sample a pushing state
        _oracle_sampler->samplePushingState(approach->getState(), target_state, t, _pushing_state_eps);
        // check if its valid or not
        if (!_si->isValid(approach->getState()))
            return false;
    }
#ifdef DEBUG_PRINTOUTS
    printState("Pushing from state ", approach->getState()); // TODO remove
#endif
    // query the policy
    _oracle_sampler->queryPolicy(push->getControl(), approach->getState(), target_state, t);
    // propagate
    return _state_propagator->propagate(approach->getState(), push->getControl(), push->getState());
}

/************************************ GreedyMultiExtendRRT **************************************/
GreedyMultiExtendRRT::GreedyMultiExtendRRT(::ompl::control::SpaceInformationPtr si,
    oracle::PushingOraclePtr pushing_oracle,
    oracle::RobotOraclePtr robot_oracle,
    const std::string& robot_name)
    : MultiExtendRRT(si, pushing_oracle, robot_oracle, robot_name)
    , _log_prefix("[mps::planner::pushing::algorithm::GreedyMultiExtendRRT::")
{
    _distances_after.resize(_state_space->getNumObjects());
    _distances_before.resize(_state_space->getNumObjects());
    _vector_a.resize(_state_space->getNumObjects());
}

GreedyMultiExtendRRT::~GreedyMultiExtendRRT() = default;

RearrangementPlanner::PlanningQueryPtr GreedyMultiExtendRRT::createPlanningQuery(mps::planner::ompl::state::goal::ObjectsRelocationGoalPtr goal_region,
    mps::planner::ompl::state::SimEnvWorldState* start_state, const std::string& robot_name, float timeout)
{
    auto pq = MultiExtendRRT::createPlanningQuery(goal_region, start_state, robot_name, timeout);
    // declare disturbance tolerance
    using std::placeholders::_1;
    std::function<void(float)> dt_setter = std::bind(&GreedyMultiExtendRRT::setDisturbanceTolerance, this, _1);
    std::function<float()> dt_getter = std::bind(&GreedyMultiExtendRRT::getDisturbanceTolerance, this);
    pq->parameters->declareParam<float>("disturbance_tol", dt_setter, dt_getter);
    pq->parameters->setParam("disturbance_tol", "0.03");
    // declare target tolerance
    std::function<void(float)> tt_setter = std::bind(&GreedyMultiExtendRRT::setTargetTolerance, this, _1);
    std::function<float()> tt_getter = std::bind(&GreedyMultiExtendRRT::getTargetTolerance, this);
    pq->parameters->declareParam<float>("target_tol", tt_setter, tt_getter);
    pq->parameters->setParam("target_tol", "0.02");
    // declare num pushing trials
    std::function<void(float)> pt_setter = std::bind(&GreedyMultiExtendRRT::setNumPushingTrials, this, _1);
    std::function<float()> pt_getter = std::bind(&GreedyMultiExtendRRT::getNumPushingTrials, this);
    pq->parameters->declareParam<float>("pushing_trials", pt_setter, pt_getter);
    pq->parameters->setParam("pushing_trials", "10");
    // // declare position tolerance
    // std::function<void(float)> pt_setter = std::bind(&GreedyMultiExtendRRT::setPositionTolerance, this, _1);
    // std::function<float()> pt_getter = std::bind(&GreedyMultiExtendRRT::getPositionTolerance, this);
    // pq->parameters->declareParam<float>("position_tol", pt_setter, pt_getter);
    // pq->parameters->setParam("position_tol", "0.02");
    // // declare orientation tolerance
    // std::function<void(float)> ot_setter = std::bind(&GreedyMultiExtendRRT::setOrientationTolerance, this, _1);
    // std::function<float()> ot_getter = std::bind(&GreedyMultiExtendRRT::getOrientationTolerance, this);
    // pq->parameters->declareParam<float>("orientation_tol", ot_setter, ot_getter);
    // pq->parameters->setParam("orientation_tol", "0.2");
    return pq;
}

bool GreedyMultiExtendRRT::extend(const SlicePtr& current, const PushMotionPtr& target_state,
    PushMotionPtr& last_motion, PlanningBlackboard& pb)
{
    static const std::string log_prefix(_log_prefix + "extend] ");
    auto target_slice = _slice_cache.getNewSlice(target_state);
    // create a set of all movable indices
    MovableSet all_movables;
    for (unsigned int m = 0; m < _state_space->getNumObjects(); ++m) {
        if (m == pb.robot_id)
            continue;
        all_movables.insert(m);
    }
    PushMotionPtr push_motion = std::dynamic_pointer_cast<PushMotion>(current->repr);
    StateSlicePair current_state_slice = std::make_pair(push_motion, current);
    // run over all movables and try recursive extend
    for (unsigned int t : all_movables) {
        auto targets = MovableSet(all_movables);
        auto remainers = MovableSet();
        targets.erase(t);
        ExtensionProgress result;
        std::tie(result, last_motion) = recursiveExtend(t, current_state_slice, targets, remainers, target_slice, pb);
        if (result == ExtensionProgress::REACHED or result == ExtensionProgress::GOAL_REACHED) {
            _slice_cache.cacheSlice(target_slice);
            return result == ExtensionProgress::GOAL_REACHED;
        }
    }
    _slice_cache.cacheSlice(target_slice);
    return false;
}

std::tuple<GreedyMultiExtendRRT::ExtensionProgress, PushMotionPtr> GreedyMultiExtendRRT::recursiveExtend(
    unsigned int t, const StateSlicePair& start, MovableSet& targets,
    MovableSet& remainers, SliceConstPtr target_slice, PlanningBlackboard& pb)
{
    MovableSet movables;
    movables.insert(remainers.begin(), remainers.end());
    movables.insert(targets.begin(), targets.end());
    StateSlicePair current = start;
    std::vector<StateSlicePair> push_path;
    MovableSet blockers;
    PushMotionPtr after_push;
    GreedyMultiExtendRRT::PushResult push_result;
    std::tie(push_result, after_push) = tryPush(t, current, movables, blockers, target_slice, true, pb);
    while (push_result == GreedyMultiExtendRRT::PushResult::PROGRESS) {
        // add new state to search tree
        auto new_slice = addToTree(after_push, current.first, pb);
        // check whether it is a goal
        if (pb.pq->goal_region->isSatisfied(after_push->getState())) {
            // if yes, abort and return that we reached it
            return { GreedyMultiExtendRRT::ExtensionProgress::GOAL_REACHED, after_push };
        }
        // continue pushing
        current = std::make_pair(after_push, new_slice);
        push_path.push_back(current);
        std::tie(push_result, after_push) = tryPush(t, current, movables, blockers, target_slice, false, pb);
    }
    // push finished, check what happened
    if (push_result == GreedyMultiExtendRRT::PushResult::REACHED || push_result == GreedyMultiExtendRRT::PushResult::POLICY_FAIL) {
        // we succeeded at pushing t to its goal (or the policy can not push it closer), try pushing the remaining targets
        for (auto t_prime : targets) {
            MovableSet sub_targets(targets);
            sub_targets.erase(t_prime);
            MovableSet sub_remainers(remainers);
            auto [sub_result, sub_final_motion] = recursiveExtend(t_prime, current, sub_targets, sub_remainers, target_slice, pb);
            // in case of success, return
            if (sub_result != GreedyMultiExtendRRT::ExtensionProgress::FAIL) {
                remainers = sub_remainers;
                return { sub_result, sub_final_motion };
            }
        }
    } else if (push_result == GreedyMultiExtendRRT::PushResult::MOVABLES_BLOCK_PUSH) {
        // movables are blocking the push, try clearing
        // create the set of objects that the subroutine may move if need be
        MovableSet sub_remainers;
        std::set_difference(movables.cbegin(), movables.cend(), blockers.cbegin(), blockers.cend(),
            std::inserter(sub_remainers, sub_remainers.begin()));
        // try to clear the blockers, attempt this by backtracking starting from the most recent state
        while (!push_path.empty()) {
            // _motion_cache.cacheMotion(current.first);
            // _slice_cache.cacheSlice(current.second);
            current = push_path.back();
            push_path.pop_back();

            // try different orders of clearing blockers
            for (auto b : blockers) {
                // the sub targets are all blockers minus the current one
                MovableSet sub_targets(blockers);
                sub_targets.erase(b);
                // MovableSet sub_remainers(movables);
                auto [sub_result, sub_final_motion] = recursiveExtend(b, current, sub_targets, sub_remainers, target_slice, pb);
                if (sub_result == GreedyMultiExtendRRT::ExtensionProgress::REACHED) {
                    // update remainers set
                    MovableSet old_remainers(remainers);
                    remainers.clear();
                    std::set_intersection(old_remainers.cbegin(), old_remainers.cend(),
                        sub_remainers.cbegin(), sub_remainers.cend(), std::inserter(remainers, remainers.begin()));
                    // update target set
                    MovableSet remaining_targets(targets);
                    std::set_intersection(targets.cbegin(), targets.cend(), sub_remainers.cbegin(), sub_remainers.cend(),
                        std::inserter(remaining_targets, remaining_targets.begin()));
                    current = std::make_pair(sub_final_motion, getSlice(sub_final_motion, pb));
                    return recursiveExtend(t, current, remaining_targets, remainers, target_slice, pb);
                } else if (sub_result == GreedyMultiExtendRRT::ExtensionProgress::GOAL_REACHED) {
                    return { sub_result, sub_final_motion };
                } // else try other blocking object first
            }
        } // if all attempts to clear the movable block failed, we fail
    }
    return { GreedyMultiExtendRRT::ExtensionProgress::FAIL, nullptr };
}

std::tuple<GreedyMultiExtendRRT::PushResult, PushMotionPtr> GreedyMultiExtendRRT::tryPush(
    unsigned int t, const StateSlicePair& current, MovableSet& movables, MovableSet& blockers,
    SliceConstPtr target_slice, bool new_push, PlanningBlackboard& pb)
{
    if (isCloseEnough(current.first, target_slice, t))
        return { PushResult::REACHED, current.first };
    blockers.clear();
    unsigned int min_num_blockers = std::numeric_limits<unsigned int>::max();
    // allocate motions
    PushMotionPtr approach_motion = _motion_cache.getNewMotion();
    PushMotionPtr pushing_motion = _motion_cache.getNewMotion();
    PushMotionPtr tmp_approach_motion = _motion_cache.getNewMotion();
    PushMotionPtr tmp_pushing_motion = _motion_cache.getNewMotion();
    // init approach motion
    _state_space->copyState(approach_motion->getState(), current.first->getState());
    // init tmp approach motion
    _state_space->copyState(tmp_approach_motion->getState(), current.first->getState());
    // cast start and target state
    auto* start_state = dynamic_cast<mps_state::SimEnvWorldState*>(current.first->getState());
    auto* target_state = dynamic_cast<mps_state::SimEnvWorldState*>(target_slice->repr->getState());
    bool no_policy_fail = false;
    // try pushing
    for (unsigned int trial = 0; trial < _num_pushing_trials; ++trial) {
        bool extension_success = samplePush(start_state, target_state, tmp_approach_motion, tmp_pushing_motion, t, trial > 0 or new_push);
        pb.stats.num_state_propagations++;
        if (!extension_success) {
            continue;
        }
#ifdef DEBUG_PRINTOUTS
        printState("Pushed to ", tmp_pushing_motion->getState()); // TODO remove
#endif
        // evaluate the result
        bool made_progress = madeProgress(tmp_approach_motion, tmp_pushing_motion, target_slice, t, _state_space);
        no_policy_fail |= made_progress; // if we made progress once, we can not blame the policy for failing
        // check if there is anything blocking the push
        MovableSet tmp_blockers;
        bool valid_blockers = getPushBlockers(tmp_approach_motion, tmp_pushing_motion, target_slice, movables, tmp_blockers, pb);
        // check whether the newly sampled push is better than what we had before
        if (valid_blockers and made_progress and blockers.size() < min_num_blockers) {
            blockers = tmp_blockers;
            min_num_blockers = blockers.size();
            _si->copyState(approach_motion->getState(), tmp_approach_motion->getState());
            _si->copyState(pushing_motion->getState(), tmp_pushing_motion->getState());
            _si->copyControl(approach_motion->getControl(), tmp_approach_motion->getControl());
            _si->copyControl(pushing_motion->getControl(), tmp_pushing_motion->getControl());
            if (min_num_blockers == 0) // we have clear push
                break;
        }
    }
    _motion_cache.cacheMotion(tmp_approach_motion);
    _motion_cache.cacheMotion(tmp_pushing_motion);
    if (min_num_blockers == std::numeric_limits<unsigned int>::max()) {
        _motion_cache.cacheMotion(approach_motion);
        _motion_cache.cacheMotion(pushing_motion);
        if (no_policy_fail) {
            return { GreedyMultiExtendRRT::PushResult::FAIL, nullptr };
        }
        return { GreedyMultiExtendRRT::PushResult::POLICY_FAIL, nullptr };
    } else if (min_num_blockers == 0) {
        // set up approach motion
        approach_motion->setTargetId(pb.robot_id);
        approach_motion->setTeleportTransit(true); // TODO use roadmap here?
        pushing_motion->setTargetId(t);
        pushing_motion->setParent(approach_motion);
        if (isCloseEnough(pushing_motion, target_slice, t)) {
            return { GreedyMultiExtendRRT::PushResult::REACHED, pushing_motion };
        } else {
            return { GreedyMultiExtendRRT::PushResult::PROGRESS, pushing_motion };
        }
    } else {
        return { GreedyMultiExtendRRT::PushResult::MOVABLES_BLOCK_PUSH, nullptr };
    }
}

bool GreedyMultiExtendRRT::isCloseEnough(const PushMotionPtr& current,
    SliceConstPtr target_slice, unsigned int t) const
{
    auto object_state_space = _state_space->getSubspace(t);
    auto current_state = dynamic_cast<mps_state::SimEnvWorldState*>(current->getState());
    auto target_state = dynamic_cast<mps_state::SimEnvWorldState*>(target_slice->repr->getState());
    auto dist = object_state_space->distance(current_state->getObjectState(t), target_state->getObjectState(t));
    return dist < _target_tolerance;
}

bool GreedyMultiExtendRRT::getPushBlockers(const PushMotionConstPtr& x_b, const PushMotionConstPtr& x_a, const SliceConstPtr& target_slice,
    const MovableSet& movables, MovableSet& blockers, PlanningBlackboard& pb) const
{
    bool valid_blockers = true;
    blockers.clear();
    computeObjectDistances(x_b, target_slice, _distances_before);
    computeObjectDistances(x_a, target_slice, _distances_after);
    _vector_a = _distances_after - _distances_before;
    for (size_t i = 0; i < _vector_a.size(); ++i) {
        if (_vector_a[i] > _disturbance_tolerance and i != pb.robot_id) {
            blockers.insert(i);
            valid_blockers = valid_blockers & movables.find(i) != movables.end();
        }
    }
    return valid_blockers;
}

void GreedyMultiExtendRRT::computeObjectDistances(const PushMotionConstPtr& x, const SliceConstPtr& target, Eigen::VectorXf& dist_array) const
{
    auto target_state = dynamic_cast<const mps_state::SimEnvWorldState*>(target->repr->getConstState());
    for (size_t i = 0; i < _state_space->getNumObjects(); ++i) {
        auto object_state_space = _state_space->getSubspace(i);
        auto x_state = dynamic_cast<const mps_state::SimEnvWorldState*>(x->getConstState());
        dist_array[i] = object_state_space->distance(x_state->getObjectState(i), target_state->getObjectState(i));
    }
}

void GreedyMultiExtendRRT::setDisturbanceTolerance(float tol)
{
    _disturbance_tolerance = tol;
}

float GreedyMultiExtendRRT::getDisturbanceTolerance() const
{
    return _disturbance_tolerance;
}

void GreedyMultiExtendRRT::setTargetTolerance(float tol)
{
    _target_tolerance = tol;
}

float GreedyMultiExtendRRT::getTargetTolerance() const
{
    return _target_tolerance;
}

void GreedyMultiExtendRRT::setNumPushingTrials(unsigned int trials)
{
    _num_pushing_trials = trials;
}

unsigned int GreedyMultiExtendRRT::getNumPushingTrials() const
{
    return _num_pushing_trials;
}

// void GreedyMultiExtendRRT::setPositionTolerance(float tol)
// {
//     _position_tolerance = tol;
// }

// float GreedyMultiExtendRRT::getPositionTolerance() const
// {
//     return _position_tolerance;
// }

// void GreedyMultiExtendRRT::setOrientationTolerance(float tol)
// {
//     _orientation_tolerance = tol;
// }

// float GreedyMultiExtendRRT::getOrientationTolerance() const
// {
//     return _orientation_tolerance;
// }

MERRTExecutionMonitor::MERRTExecutionMonitor(MultiExtendRRTPtr planner, ExecutionCallback excall)
    : ExecutionMonitor(planner, excall)
    , _merrt_planner(planner)
    , _si(_planner->getSpaceInformation())
    , _propagator(std::dynamic_pointer_cast<mps_control::SimEnvStatePropagator>(_si->getStatePropagator()))
    , _state_space(std::dynamic_pointer_cast<mps_state::SimEnvWorldStateSpace>(_si->getStateSpace()))
    , _motion_cache(_si)
    , _transfer_tolerance(0.01)
    , _num_pushing_trials(10)
{
    assert(_propagator);
    assert(_state_space);
}

MERRTExecutionMonitor::MERRTExecutionMonitor(MultiExtendRRTPtr planner)
    : MERRTExecutionMonitor(planner, ExecutionCallback())
{
}

MERRTExecutionMonitor::~MERRTExecutionMonitor() = default;

bool MERRTExecutionMonitor::execute(RearrangementPlanner::PlanningQueryPtr pq)
{
    static const std::string log_prefix("[mps::planner::pushing::algorithm::MERRTExecutionMonitor::execute]");
    if (pq->path == nullptr) {
        logging::logWarn("There is no solution to execute.", log_prefix);
        return false;
    }
    _robot_state_space = _state_space->getObjectStateSpace(pq->robot_name);
    _robot_id = _state_space->getSubspaceIndex(pq->robot_name);
    auto s = dynamic_cast<mps_state::SimEnvWorldState*>(_si->allocState());
    auto path = pq->path->deepCopy();
    // execute the first motion
    bool no_error = _excall(path->getMotion(0), s);
    path = path->getSubPath(1); // TODO replace by pop front
    while (path->getNumMotions() and no_error) {
        if (updatePath(path, s, pq)) {
            no_error = _excall(path->getMotion(0), s);
            if (!no_error)
                break;
            path = path->getSubPath(1); // TODO replace by pop front
        } else {
            logging::logWarn("Execution significantly deviated from solution, need to replan", log_prefix);
            // TODO replan
            path->clear(); // TODO remove
            break;
        }
    }
    bool goal_reached = pq->goal_region->isSatisfied(s);
    _si->freeState(s);
    return no_error and goal_reached;
}

float MERRTExecutionMonitor::getTransferTolerance() const
{
    return _transfer_tolerance;
}

void MERRTExecutionMonitor::setTransferTolerance(float val)
{
    _transfer_tolerance = val;
}

void MERRTExecutionMonitor::segmentPath(mps::planner::ompl::planning::essentials::PathPtr intended_path,
    mps::planner::ompl::planning::essentials::PathPtr predicted_path,
    MERRTExecutionMonitor::SegmentedPath& segments) const
{
    PushMotionPtr motion = std::dynamic_pointer_cast<PushMotion>(intended_path->getMotion(0));
    unsigned int prev_target = motion->getTargetId();
    segments.clear();
    segments.push_back(std::make_pair(TransitSegment(), TransferSegment()));

    for (unsigned int i = 0; i < intended_path->getNumMotions(); ++i) {
        PushMotionPtr current_motion = std::dynamic_pointer_cast<PushMotion>(intended_path->getMotion(i));
        unsigned int current_target = current_motion->getTargetId();
        if (current_target != prev_target and current_target == _robot_id) { // change in target from transfer to transit
            segments.push_back(std::make_pair(TransitSegment(), TransferSegment()));
        }
        if (current_target == _robot_id) {
            segments.back().first.intended.push_back(current_motion);
            // predicted path is only extended as long as it goes, i.e. is valid
            if (predicted_path->getNumMotions() > i) {
                segments.back().first.predicted.push_back(std::dynamic_pointer_cast<PushMotion>(predicted_path->getMotion(i)));
            }
        } else {
            segments.back().second.target_id = current_target;
            segments.back().second.intended.push_back(current_motion);
            if (predicted_path->getNumMotions() > i) {
                segments.back().second.predicted.push_back(std::dynamic_pointer_cast<PushMotion>(predicted_path->getMotion(i)));
            }
        }
        prev_target = current_target;
    }
}

bool MERRTExecutionMonitor::updatePath(PathPtr path, mps_state::SimEnvWorldState* s, RearrangementPlanner::PlanningQueryPtr pq)
{
    static const std::string log_prefix("[mps::planner::pushing::algorithm::MERRTExecutionMonitor::updatePath]");
    PathPtr predicted_path = std::make_shared<Path>(_si);
    if (_planner->isGoalPath(path, s, pq, predicted_path)) {
        return true;
    }
    if (!_si->isValid(s)) {
        logging::logWarn("Can not update path. Current state is invalid.", log_prefix);
        return false;
    }
    // the path is no longer valid, try to save it
    // create tmp state
    ::ompl::base::ScopedState<mps_state::SimEnvWorldStateSpace> tmp_state(_si);
    mps_state::SimEnvWorldState* curr_predicted_state = s;
    PushMotionPtr prev_motion = nullptr;
    // segment path
    SegmentedPath segments;
    segmentPath(path, predicted_path, segments);
    // bool path_fixed = false;
    // run over segments and try to figure out which one fail and see whether we can fix it
    for (auto iter = segments.begin(); iter != segments.end(); ++iter) {
        auto& [transit, transfer] = *iter;
        bool need_new_transfer = false;
        // do we have a transit?
        if (!transit.intended.empty()) {
            // get pushing state (i.e. target of transit)
            auto* intended_state = dynamic_cast<const mps_state::SimEnvWorldState*>(transit.intended.back()->getConstState());
            // set the robot to its pushing state
            _state_space->copyState(tmp_state.get(), curr_predicted_state);
            _state_space->copySubState(tmp_state.get(), intended_state, _robot_id);
            // is the pushing state still valid in our predicted world state?
            if (_si->isValid(tmp_state.get())) {
                // check whether we would reach the goal if this transit was valid and took the robot to the pushing state
                // TODO implement in case we have actual transit motions
                // bool would_reach_goal = _planner->isGoalPath(path->getSubPath(transfer.start_id), tmp_state, pq, tmp_new_path);
                // if (would_reach_goal) {
                //     // this transit breaks the solution, thus replan it
                //     bool replan_success = replan_transit(transit);
                //     if (replan_success) {
                //         // TODO update path
                //         return true;
                //     }
                //     need_new_transfer = true;
                // }
            } else {
                need_new_transfer = true;
            }
        }
        // check whether we need a new transfer
        need_new_transfer |= transfer.predicted.empty();
        if (!need_new_transfer) { // if we already know, no need to check more
            // else check whether we reach the desired target state of the transfer
            auto* intended_state = dynamic_cast<const mps_state::SimEnvWorldState*>(transfer.intended.back()->getConstState());
            auto* predicted_state = dynamic_cast<const mps_state::SimEnvWorldState*>(transfer.predicted.back()->getConstState());
            float transfer_error = _robot_state_space->distance(intended_state->getObjectState(transfer.target_id),
                predicted_state->getObjectState(transfer.target_id));
            need_new_transfer = transfer_error > _transfer_tolerance;
        }
        if (need_new_transfer) {
            if (updateTransfer(curr_predicted_state, transit, transfer, prev_motion)) {
                // update predicted path and check whether we reach a goal now
                if (updatePathPrediction(segments, iter, pq)) {
                    extractNewPath(segments, path);
                    return true;
                } // else continue
            } else {
                return false;
            }
        }
        prev_motion = std::dynamic_pointer_cast<PushMotion>(transfer.predicted.back());
        curr_predicted_state = dynamic_cast<mps_state::SimEnvWorldState*>(prev_motion->getState());
    }
    return false;
}

void MERRTExecutionMonitor::extractNewPath(SegmentedPath& segments, mps::planner::ompl::planning::essentials::PathPtr path) const
{
    path->clear();
    for (auto& [transit, transfer] : segments) {
        for (auto& motion : transit.predicted) {
            path->append(motion);
        }
        for (auto& motion : transfer.predicted) {
            path->append(motion);
        }
    }
}

bool MERRTExecutionMonitor::updatePathPrediction(SegmentedPath& segments, const SegmentedPath::iterator& iter,
    RearrangementPlanner::PlanningQueryPtr pq)
{
    // run over the actions past iter and update the prediction
    // in this process check whether we reached a goal or the path became invalid
    auto& [last_transit, last_transfer] = *iter;
    PushMotionPtr prev_motion = last_transfer.predicted.back();
    for (auto my_iter = iter + 1; my_iter != segments.end(); ++my_iter) {
        auto& [current_transit, current_transfer] = *my_iter;
        PredictionUpdate pu = simulateSegment(current_transit.intended, current_transit.predicted, prev_motion, pq);
        switch (pu) {
        case PredictionUpdate::INVALID:
            return false;
        case PredictionUpdate::GOAL_REACHED:
            // remove rest of the path
            // TODO Does reaching the goal after a transit break assumptions in path segmentation???
            segments.erase(my_iter + 1, segments.end());
            _motion_cache.cacheMotions(current_transfer.predicted);
            return true;
        case PredictionUpdate::VALID:
            // check the following transfer motion
            break;
        }
        pu = simulateSegment(current_transfer.intended, current_transfer.predicted, prev_motion, pq);
        switch (pu) {
        case PredictionUpdate::INVALID:
            return false;
        case PredictionUpdate::GOAL_REACHED:
            // remove rest of the path
            // TODO Does reaching the goal after a transit break assumptions in path segmentation???
            segments.erase(my_iter + 1, segments.end());
            return true;
        case PredictionUpdate::VALID:
            // check next actions
            break;
        }
    }
    return false;
}

MERRTExecutionMonitor::PredictionUpdate MERRTExecutionMonitor::simulateSegment(const std::vector<PushMotionConstPtr>& intended, std::vector<PushMotionPtr>& predicted,
    PushMotionPtr& prev_motion, RearrangementPlanner::PlanningQueryPtr pq)
{
    // run through all intended motions and simulate them updating predicted motions
    for (unsigned int midx = 0; midx < intended.size(); ++midx) {
        auto intended_motion = std::dynamic_pointer_cast<const PushMotion>(intended[midx]);
        PushMotionPtr predict_motion;
        if (predicted.size() > midx) {
            predict_motion = predicted[midx];
        } else {
            predict_motion = _motion_cache.getNewMotion();
            predict_motion->setParent(prev_motion);
            predicted.push_back(predict_motion);
        }
        predict_motion->setTargetId(intended_motion->getTargetId());
        // is this a teleport action? then teleport robot, else propagate action
        bool valid = false;
        if (intended_motion->isTeleportTransit()) {
            predict_motion->setTeleportTransit(true);
            // copy robot state
            _state_space->copySubState(predict_motion->getState(), intended_motion->getConstState(), _robot_id);
            valid = _si->isValid(predict_motion->getState());
        } else {
            // propagate action
            _si->copyControl(predict_motion->getControl(), intended_motion->getConstControl());
            valid = _propagator->propagate(prev_motion->getConstState(), predict_motion->getControl(), predict_motion->getState());
        }
        if (!valid) {
            // free left over predicted motions
            _motion_cache.cacheMotions(predicted, predicted.begin() + midx, predicted.end());
            return PredictionUpdate::INVALID;
        }
        // check whether we have a goal
        if (pq->goal_region->isSatisfied(predict_motion->getState())) {
            // erase predicted motions after this, if any
            if (predicted.size() > midx)
                return PredictionUpdate::GOAL_REACHED;
        }
        prev_motion = predict_motion;
    }
    return PredictionUpdate::VALID;
}

bool MERRTExecutionMonitor::updateTransfer(const mps_state::SimEnvWorldState* start,
    TransitSegment& transit, TransferSegment& transfer, PushMotionPtr prev_motion)
{
    // get target_state
    auto target_state = dynamic_cast<const mps_state::SimEnvWorldState*>(transfer.intended.back()->getConstState());
    // try to compute a new push
    auto [approach_motion, push_motion] = tryPush(transfer.target_id, start, target_state);
    if (approach_motion != nullptr and push_motion != nullptr) {
        // we have a new transfer and transit
        // add transit (this may be a path)
        transit.predicted.clear();
        while (approach_motion != nullptr) {
            transit.predicted.push_back(approach_motion);
            approach_motion = std::dynamic_pointer_cast<PushMotion>(approach_motion->getParent());
        }
        std::reverse(transit.predicted.begin(), transit.predicted.end());
        transit.predicted.front()->setParent(prev_motion);
        // set transfer
        transfer.predicted.clear();
        transfer.predicted.push_back(push_motion);
        return true;
    }
    return false;
}

std::tuple<PushMotionPtr, PushMotionPtr> MERRTExecutionMonitor::tryPush(
    unsigned int t, const mps_state::SimEnvWorldState* start, const mps_state::SimEnvWorldState* goal)
{
    bool success = false;
    float best_distance = _state_space->objectStateDistance(start, goal, t);
    // allocate motions
    PushMotionPtr approach_motion = _motion_cache.getNewMotion();
    PushMotionPtr pushing_motion = _motion_cache.getNewMotion();
    PushMotionPtr tmp_approach_motion = _motion_cache.getNewMotion();
    PushMotionPtr tmp_pushing_motion = _motion_cache.getNewMotion();
    auto* resulting_state = dynamic_cast<mps_state::SimEnvWorldState*>(tmp_pushing_motion->getState());
    // try pushing
    for (unsigned int trial = 0; trial < _num_pushing_trials; ++trial) {
        bool push_valid = _merrt_planner->samplePush(start, goal, tmp_approach_motion, tmp_pushing_motion, t, true);
        // TODO here we would need to verify that a transit is possible
        if (push_valid) {
            float dist = _state_space->objectStateDistance(resulting_state, goal, t);
            if (dist < best_distance) {
                // copy approach and pushing motion
                _state_space->copyState(approach_motion->getState(), tmp_approach_motion->getState());
                // _si->copyControl(approach_motion->getControl(), tmp_approach_motion->getControl());
                _state_space->copyState(pushing_motion->getState(), tmp_pushing_motion->getState());
                _si->copyControl(pushing_motion->getControl(), tmp_pushing_motion->getControl());
                success = true;
            }
        }
    }
    _motion_cache.cacheMotion(tmp_approach_motion);
    _motion_cache.cacheMotion(tmp_pushing_motion);
    // init
    approach_motion->setTeleportTransit(true);
    if (!success) {
        _motion_cache.cacheMotion(approach_motion);
        _motion_cache.cacheMotion(pushing_motion);
        return { nullptr, nullptr };
    } else {
        // set up approach motion
        approach_motion->setTargetId(_robot_id);
        approach_motion->setTeleportTransit(true); // TODO set differently if using roadmap
        pushing_motion->setTargetId(t);
        pushing_motion->setParent(approach_motion);
        return { approach_motion, pushing_motion };
    }
}