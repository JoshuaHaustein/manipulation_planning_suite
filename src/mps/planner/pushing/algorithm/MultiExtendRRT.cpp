#include <algorithm>
#include <mps/planner/pushing/algorithm/MultiExtendRRT.h>
#include <ompl/datastructures/NearestNeighborsGNAT.h>

namespace logging = mps::planner::util::logging;
namespace ob = ::ompl::base;
namespace oc = ::ompl::control;
namespace mps_state = mps::planner::ompl::state;
namespace mps_control = mps::planner::ompl::control;
namespace mps_oracle = mps::planner::pushing::oracle;
using namespace mps::planner::pushing::algorithm;
using namespace mps::planner::ompl::planning::essentials;

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
    _slices_nn->clear();
    _slice_distance_fn->distance_measure.setWeights(pq->weights);
    _slice_distance_fn->setRobotId(blackboard.robot_id);
    _robot_state_dist_fn->setRobotId(blackboard.robot_id);
    _slices_nn->clear();
    _pushing_oracle->timer = _timer;
    logging::logInfo("MultiExtendRRT setup for the following query:\n " + blackboard.pq->toString(), log_prefix);
}

void MultiExtendRRT::addToTree(PushMotionPtr new_motion, PushMotionPtr root, PlanningBlackboard& pb)
{
    std::vector<MotionPtr> path;
    MotionPtr motion = new_motion;
    while (motion != nullptr) {
        path.push_back(motion);
        motion = motion->getParent();
    }
    path.back()->setParent(root);
    for (auto iter = path.rbegin(); iter != path.rend(); iter++) {
        // _tree->add(new_motion);
        if (_debug_drawer) {
            _debug_drawer->addNewMotion(*iter);
        }
        SlicePtr closest_slice = getSlice(*iter, pb);
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
        addToTree(after_push, current.first, pb);
        // check whether it is a goal
        if (pb.pq->goal_region->isSatisfied(after_push->getState())) {
            // if yes, abort and return that we reached it
            return { GreedyMultiExtendRRT::ExtensionProgress::GOAL_REACHED, after_push };
        }
        // continue pushing
        current = std::make_pair(after_push, getSlice(after_push, pb));
        push_path.push_back(current);
        std::tie(push_result, after_push) = tryPush(t, current, movables, blockers, target_slice, false, pb);
    }
    // push finished, check what happened
    if (push_result == GreedyMultiExtendRRT::PushResult::REACHED) {
        // we succeeded at pushing t to its goal, try pushing the remaining targets
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
        MovableSet sub_remainers(movables);
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
                    // uipdate target set
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
    // free motions
    for (auto pair : push_path) {
        _motion_cache.cacheMotion(pair.first);
        _slice_cache.cacheSlice(pair.second);
    }
    if (after_push)
        _motion_cache.cacheMotion(after_push);
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
    // try pushing
    for (unsigned int trial = 0; trial < _num_pushing_trials; ++trial) {
        if (trial > 0 or new_push) {
            // sample a pushing state
            _oracle_sampler->sampleFeasibleState(tmp_approach_motion->getState(), target_slice->repr->getState(), t, _pushing_state_eps);
            // check if its valid or not
            if (!_si->isValid(tmp_approach_motion->getState()))
                continue;
        }
        // query the policy
        _oracle_sampler->queryPolicy(tmp_pushing_motion->getControl(), tmp_approach_motion->getState(), target_slice->repr->getState(), t);
        // propagate
        bool extension_success = _state_propagator->propagate(tmp_approach_motion->getState(),
            tmp_pushing_motion->getControl(),
            tmp_pushing_motion->getState());
        pb.stats.num_state_propagations++;
        if (!extension_success) {
            continue;
        }
        // evaluate the result
        bool made_progress = madeProgress(tmp_approach_motion, tmp_pushing_motion, target_slice, t);
        MovableSet tmp_blockers;
        bool valid_blockers = getPushBlockers(tmp_approach_motion, tmp_pushing_motion, target_slice, movables, tmp_blockers, pb);
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
        return { GreedyMultiExtendRRT::PushResult::FAIL, nullptr };
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

bool GreedyMultiExtendRRT::madeProgress(PushMotionPtr x_b, PushMotionPtr x_a, SliceConstPtr target_slice, unsigned int t) const
{
    auto object_state_space = _state_space->getSubspace(t);
    auto before_state = dynamic_cast<mps_state::SimEnvWorldState*>(x_b->getState());
    auto after_state = dynamic_cast<mps_state::SimEnvWorldState*>(x_a->getState());
    auto target_state = dynamic_cast<mps_state::SimEnvWorldState*>(target_slice->repr->getState());
    float dist_before = object_state_space->distance(before_state->getObjectState(t), target_state->getObjectState(t));
    float dist_after = object_state_space->distance(after_state->getObjectState(t), target_state->getObjectState(t));
    return dist_after < dist_before;
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
