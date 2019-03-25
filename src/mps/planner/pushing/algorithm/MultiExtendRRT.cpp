#include <mps/planner/pushing/algorithm/MultiExtendRRT.h>
#include <ompl/datastructures/NearestNeighborsGNAT.h>

namespace logging = mps::planner::util::logging;
namespace ob = ::ompl::base;
namespace oc = ::ompl::control;
namespace mps_state = mps::planner::ompl::state;
namespace mps_control = mps::planner::ompl::control;
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

/************************************ GreedyMonotoneExtension **************************************/
GreedyMonotoneExtension::GreedyMonotoneExtension(oracle::PushingOraclePtr pushing_oracle, oracle::RobotOraclePtr robot_oracle)
{
    // TODO
}

GreedyMonotoneExtension::~GreedyMonotoneExtension() = default;

void GreedyMonotoneExtension::setup(unsigned int robot_id, float goal_tolerance, float disturbance_tolerance)
{
    // TODO
}

GreedyMonotoneExtension::ExtensionProgress GreedyMonotoneExtension::extend(SlicePtr start_slice,
    mps::planner::ompl::state::SimEnvWorldState const* target,
    std::vector<PushMotionPtr>& path)
{
    // TODO
    return GreedyMonotoneExtension::ExtensionProgress::FAIL;
}

std::tuple<GreedyMonotoneExtension::ExtensionProgress, PushMotionPtr> GreedyMonotoneExtension::recursiveExtend(
    unsigned int t, PushMotionPtr x_c, MovableSet& targets,
    MovableSet& remainers, mps::planner::ompl::state::SimEnvWorldState const* target_slice)
{
    // TODO
    return { GreedyMonotoneExtension::ExtensionProgress::FAIL, nullptr };
}

std::tuple<GreedyMonotoneExtension::PushResult, PushMotionPtr> GreedyMonotoneExtension::tryPush(
    unsigned int t, PushMotionConstPtr x_c, MovableSet& movables, MovableSet& blockers,
    mps::planner::ompl::state::SimEnvWorldState const* target_slice, bool new_push)
{
    // TODO
    return { GreedyMonotoneExtension::PushResult::FAIL, nullptr };
}

bool GreedyMonotoneExtension::isCloseEnough(PushMotionPtr x_c,
    mps::planner::ompl::state::SimEnvWorldState const* goal,
    unsigned int t) const
{
    // TODO
    return false;
}

bool GreedyMonotoneExtension::madeProgress(PushMotionPtr x_b, PushMotionPtr x_a, mps::planner::ompl::state::SimEnvWorldState const* goal, unsigned int t) const
{
    // TODO
    return false;
}
void GreedyMonotoneExtension::getPushBlockers(PushMotionPtr x_b, PushMotionPtr x_a, mps::planner::ompl::state::SimEnvWorldState const* goal)
{
    // TODO
}

/**************************************** MultiExtendRRT *******************************************/
MultiExtendRRT::MultiExtendRRT(oc::SpaceInformationPtr si, oracle::PushingOraclePtr pushing_oracle,
    oracle::RobotOraclePtr robot_oracle, const std::string& robot_name)
    : RearrangementPlanner(si)
    , _robot_state_dist_fn(std::make_shared<RobotStateDistanceFn>(_state_space))
    , _slice_distance_fn(std::make_shared<ObjectArrangementDistanceFn>(_state_space))
    , _pushing_oracle(pushing_oracle)
    , _min_slice_distance(0.0)
    , _goal_bias(0.1)
    , _motion_cache(si)
    , _slice_cache(_robot_state_dist_fn)
    , _log_prefix("[mps::planner::pushing::algorithm::MultiExtendRRT::")
{
    _slices_nn = std::make_shared<::ompl::NearestNeighborsGNAT<SlicePtr>>();
    _slices_nn->setDistanceFunction(std::bind(&ObjectArrangementDistanceFn::distance,
        std::ref(_slice_distance_fn),
        std::placeholders::_1,
        std::placeholders::_2));
    _rng = mps::planner::util::random::getDefaultRandomGenerator();
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
        bool goal_sampled = sample(sample_motion, blackboard);
#ifdef DEBUG_PRINTOUTS
        printState("Sampled arrangement is ", sample_motion->getState());
#endif
        // select the slice to expand
        select(sample_motion, current_slice, blackboard);
#ifdef DEBUG_PRINTOUTS
        printState("Selected tree state: ", current_motion->getState()); // TODO remove
#endif
        // Extend the tree
        solved = extend(current_slice, sample_motion->getState(), final_motion, blackboard);
#ifdef DEBUG_PRINTOUTS
        printState("Tree extended to ", final_motion->getState()); // TODO remove
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

bool MultiExtendRRT::sample(const PushMotionPtr& sample, PlanningBlackboard& pb)
{
    static const std::string log_prefix("mps::planner::pushing::algorithm::MultiExtendRRT::sample]");
    bool is_goal = false;
    // sample random state with goal biasing
    if (_rng->uniform01() < _goal_bias && pb.pq->goal_region->canSample()) {
        logging::logDebug("Sampling a goal arrangement", log_prefix);
        pb.pq->goal_region->sampleGoal(sample->getState());
        is_goal = true;
    } else {
        logging::logDebug("Sampling a random arrangement", log_prefix);
        _state_sampler->sample(sample->getState());
    }
    pb.stats.num_samples++;
    return is_goal;
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

bool MultiExtendRRT::extend(const SlicePtr& current_slice, ::ompl::base::State* dest,
    PushMotionPtr& last_motion, PlanningBlackboard& pb)
{
    // TODO use GreedyMonotoneExtension
    return false;
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

void MultiExtendRRT::addToTree(PushMotionPtr new_motion, PushMotionPtr parent, PlanningBlackboard& pb)
{
    new_motion->setParent(parent);
    // _tree->add(new_motion);
    if (_debug_drawer) {
        _debug_drawer->addNewMotion(new_motion);
    }
    SlicePtr closest_slice = getSlice(new_motion, pb);
    float slice_distance = distanceToSlice(new_motion, closest_slice);
    if (slice_distance > _min_slice_distance) { // we discovered a new slice!
        auto new_slice = _slice_cache.getNewSlice(new_motion);
        _slices_nn->add(new_slice);
        if (_debug_drawer) {
            _debug_drawer->addNewSlice(new_slice);
        }
    } else { // the new motion/state is in the same slice
        closest_slice->addSample(new_motion);
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
