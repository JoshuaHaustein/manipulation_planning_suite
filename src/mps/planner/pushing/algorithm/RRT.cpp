//
// Created by joshua on 8/14/17.
//

#include <mps/planner/ompl/control/Interfaces.h>
#include <mps/planner/pushing/algorithm/RRT.h>
#include <mps/planner/util/Logging.h>
// #define GNAT_SAMPLER
#include <ompl/datastructures/NearestNeighborsGNAT.h>

#include <boost/functional/hash.hpp>
#include <functional>
#include <queue>

namespace logging = mps::planner::util::logging;
namespace ob = ::ompl::base;
namespace oc = ::ompl::control;
namespace mps_state = mps::planner::ompl::state;
namespace mps_control = mps::planner::ompl::control;
using namespace mps::planner::pushing::algorithm;
using namespace mps::planner::ompl::planning::essentials;

////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// PlanningBlackboard ////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
RearrangementRRT::PlanningBlackboard::PlanningBlackboard(PlanningQueryPtr pq)
    : pq(pq)
    , robot_id(0)
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////// RearrangementRRT /////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
RearrangementRRT::RearrangementRRT(::ompl::control::SpaceInformationPtr si)
    : _si(si)
    , _motion_cache(si)
    , _log_prefix("[mps::planner::pushing::algorithm::RearrangementRRT::")
{
    _state_space = std::dynamic_pointer_cast<mps_state::SimEnvWorldStateSpace>(_si->getStateSpace());
    if (!_state_space) {
        throw std::logic_error(_log_prefix + "setup] Could not cast state space to SimEnvWorldStateSpace");
    }
    _distance_measure = std::make_shared<mps::planner::ompl::state::SimEnvWorldStateDistanceMeasure>(_state_space);
    _state_space->setDistanceMeasure(_distance_measure);
    // set up our search tree
    if (!_tree) {
        _tree = std::make_shared<::ompl::NearestNeighborsGNAT<mps::planner::ompl::planning::essentials::MotionPtr>>();
        // _tree = std::make_shared<::ompl::NearestNeighborsSqrtApprox <mps::planner::ompl::planning::essentials::MotionPtr> >();
        using namespace std::placeholders;
        _tree->setDistanceFunction(std::bind(&RearrangementRRT::treeDistanceFunction, this, _1, _2));
    }
    // _state_sampler = _si->allocStateSampler();
    _state_sampler = _si->allocValidStateSampler();
    assert(_state_space->getNumObjects() > 1);
    _rng = mps::planner::util::random::getDefaultRandomGenerator();
}

RearrangementRRT::~RearrangementRRT() = default;

void RearrangementRRT::setup(PlanningQueryPtr pq, PlanningBlackboard& blackboard)
{
    static const std::string log_prefix(_log_prefix + "setup]");
    setupBlackboard(blackboard);
    _distance_measure->setAll(true);
    _distance_measure->setWeights(pq->weights);
    if (_debug_drawer) {
        _debug_drawer->clear();
    }
    _tree->clear();
    logging::logInfo("Planner setup for the following query:\n " + blackboard.pq->toString(), log_prefix);
}

bool RearrangementRRT::plan(PlanningQueryPtr pq)
{
    PlanningStatistics stats;
    return plan(pq, stats);
}

bool RearrangementRRT::plan(PlanningQueryPtr pq, PlanningStatistics& stats)
{
    static const std::string log_prefix(_log_prefix + "plan]");
    PlanningBlackboard blackboard(pq);
    setup(pq, blackboard);
    logging::logDebug("Starting to plan", log_prefix);

    MotionPtr current_motion = _motion_cache.getNewMotion();
    MotionPtr sample_motion = _motion_cache.getNewMotion();
    MotionPtr final_motion = nullptr;
    // set the state to be the start state
    _si->copyState(current_motion->getState(), pq->start_state);
    // set the control to be null
    _si->nullControl(current_motion->getControl());
    // Initialize the tree
    addToTree(current_motion, nullptr, blackboard);
    final_motion = current_motion;

    bool solved = pq->goal_region->isSatisfied(current_motion->getState());
    std::stringstream ss;
    pq->goal_region->print(ss);
    logging::logDebug("Planning towards goal " + ss.str(), log_prefix);
    logging::logDebug("Entering main loop", log_prefix);
    timer_ptr->startTimer(pq->time_out);
    // Do the actual planning
    while (not timer_ptr->timeOutExceeded() and not pq->stopping_condition() && !solved) {
        blackboard.stats.num_iterations++;
        // sample a new state
        unsigned int active_obj_id = 0;
        bool goal_sampled = sample(sample_motion, active_obj_id, blackboard);
#ifdef DEBUG_PRINTOUTS
        printState("Sampled state is ", sample_motion->getState()); // TODO remove
#endif
        // Get a tree node to expand
        selectTreeNode(sample_motion, current_motion, active_obj_id, goal_sampled, blackboard);
#ifdef DEBUG_PRINTOUTS
        printState("Selected tree state: ", current_motion->getState()); // TODO remove
#endif
        logging::logDebug(boost::format("Active object is %i") % active_obj_id, log_prefix);
        // Extend the tree
        solved = extend(current_motion, sample_motion->getState(), active_obj_id, final_motion, blackboard);
#ifdef DEBUG_PRINTOUTS
        printState("Tree extended to ", final_motion->getState()); // TODO remove
#endif
    }

    blackboard.stats.runtime = timer_ptr->stopTimer();
    blackboard.stats.success = solved;
    ss.str("");
    blackboard.stats.print(ss);
    logging::logDebug("Main loop finished, stats:\n" + ss.str(),
        log_prefix);
    // create the path if we found a solution
    if (solved) {
        logging::logInfo("Found a solution", log_prefix);
        pq->path = std::make_shared<mps::planner::ompl::planning::essentials::Path>(_si);
        pq->path->initBacktrackMotion(final_motion);
    }
    // clean up
    _tree->clear();

    logging::logInfo("Planning finished", log_prefix);
    stats = blackboard.stats;
    return solved;
}

RearrangementPlanner::PlanningQueryPtr RearrangementRRT::createPlanningQuery(
    mps_state::goal::ObjectsRelocationGoalPtr goal_region,
    mps::planner::ompl::state::SimEnvWorldState* start_state,
    const std::string& robot_name,
    float timeout)
{
    auto pq = RearrangementPlanner::createPlanningQuery(goal_region, start_state, robot_name, timeout);
    using std::placeholders::_1;
    // declare goal bias
    std::function<void(float)> gb_setter = std::bind(&RearrangementRRT::setGoalBias, this, _1);
    std::function<float()> gb_getter = std::bind(&RearrangementRRT::getGoalBias, this);
    pq->parameters->declareParam<float>("goal_bias", gb_setter, gb_getter);
    pq->parameters->setParam("goal_bias", "0.2");
    // declare target bias
    std::function<void(float)> tb_setter = std::bind(&RearrangementRRT::setTargetBias, this, _1);
    std::function<float()> tb_getter = std::bind(&RearrangementRRT::getTargetBias, this);
    pq->parameters->declareParam<float>("target_bias", tb_setter, tb_getter);
    pq->parameters->setParam("target_bias", "0.25");
    // declare robot bias
    std::function<void(float)> rb_setter = std::bind(&RearrangementRRT::setRobotBias, this, _1);
    std::function<float()> rb_getter = std::bind(&RearrangementRRT::getRobotBias, this);
    pq->parameters->declareParam<float>("robot_bias", rb_setter, rb_getter);
    pq->parameters->setParam("robot_bias", "0.0");
    return pq;
}

bool RearrangementRRT::sample(mps::planner::ompl::planning::essentials::MotionPtr motion,
    unsigned int& target_obj_id,
    PlanningBlackboard& pb)
{
    static const std::string log_prefix("mps::planner::pushing::algorithm::RearrangementRRT::sample]");
    bool is_goal = false;
    // sample random state with goal biasing
    if (_rng->uniform01() < _goal_bias && pb.pq->goal_region->canSample()) {
        logging::logDebug("Sampling a goal state", log_prefix);
        pb.pq->goal_region->sampleGoal(motion->getState());
        target_obj_id = pb.pq->goal_region->sampleTargetObjectIndex();
        is_goal = true;
    } else {
        logging::logDebug("Sampling a state uniformly", log_prefix);
        _state_sampler->sample(motion->getState());
        target_obj_id = sampleActiveObject(pb);
    }
    pb.stats.num_samples++;
    return is_goal;
}

void RearrangementRRT::selectTreeNode(const ompl::planning::essentials::MotionPtr& sample_motion,
    ompl::planning::essentials::MotionPtr& selected_node,
    unsigned int& active_obj_id,
    bool sample_is_goal,
    PlanningBlackboard& pb)
{
    static const std::string log_prefix("[mps::planner::pushing::algorithm::RearrangementRRT::selectTreeNode]");
    logging::logDebug("Searching for nearest neighbor.", log_prefix);
    // select nearest state using full state space distance function
    _distance_measure->setAll(true); // we take the full state into account here
    selected_node = _tree->nearest(sample_motion);
    pb.stats.num_nearest_neighbor_queries++;
}

float RearrangementRRT::getGoalBias() const
{
    return _goal_bias;
}
float RearrangementRRT::getTargetBias() const
{
    return _target_bias;
}
float RearrangementRRT::getRobotBias() const
{
    return _robot_bias;
}

void RearrangementRRT::setGoalBias(float v)
{
    _goal_bias = v;
}

void RearrangementRRT::setTargetBias(float v)
{
    _target_bias = v;
}

void RearrangementRRT::setRobotBias(float v)
{
    _robot_bias = v;
}

void RearrangementRRT::addToTree(mps::planner::ompl::planning::essentials::MotionPtr new_motion,
    mps::planner::ompl::planning::essentials::MotionPtr parent,
    PlanningBlackboard& pb)
{
    new_motion->setParent(parent);
    _tree->add(new_motion);
    // TODO either remove again or add macro to only add this when build with debug flags
    if (_debug_drawer) {
        _debug_drawer->addNewMotion(new_motion);
    }
}

unsigned int RearrangementRRT::sampleActiveObject(const PlanningBlackboard& pb) const
{
    double random_value = _rng->uniform01();
    if (random_value < _target_bias) {
        return pb.pq->goal_region->sampleTargetObjectIndex();
    } else if (random_value < _target_bias + _robot_bias) {
        return pb.robot_id;
    } else {
        int value = _rng->uniformInt(0, _state_space->getNumObjects() - 1);
        return static_cast<unsigned int>(value);
    }
}

void RearrangementRRT::printState(const std::string& msg, ::ompl::base::State* state) const
{
    std::stringstream ss;
    auto* world_state = dynamic_cast<mps_state::SimEnvWorldState*>(state);
    ss.str("");
    ss << msg;
    world_state->print(ss);
    logging::logDebug(ss.str(), "[mps::planner::pushing::oracle::RearrangementRRT::printState]");
#ifdef DEBUG_VISUALIZE
    if (_debug_drawer)
        _debug_drawer->showState(world_state, _state_space);
#endif
}

double RearrangementRRT::treeDistanceFunction(const MotionPtr& a, const MotionPtr& b) const
{
    auto* state_a = a->getState();
    auto* state_b = b->getState();
    return _state_space->distance(state_a, state_b); // this uses _distance_measure
}

void RearrangementRRT::setDebugDrawer(DebugDrawerPtr debug_drawer)
{
    _debug_drawer = debug_drawer;
}

void RearrangementRRT::setupBlackboard(PlanningBlackboard& pb)
{
    pb.robot_id = 0;
    {
        int tmp_robot_id = _state_space->getObjectIndex(pb.pq->robot_name);
        if (tmp_robot_id < 0) {
            throw std::logic_error("[mps::planner::pushing::oracle::RearrangementRRT::setupBlackboard]"
                                   "Could not retrieve id for robot "
                + pb.pq->robot_name);
        }
        pb.robot_id = (unsigned int)tmp_robot_id;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// NaiveRearrangementRRT /////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
NaiveRearrangementRRT::NaiveRearrangementRRT(::ompl::control::SpaceInformationPtr si,
    unsigned int k)
    : RearrangementRRT(si)
    , _control_sampler(si.get(), k)
{
}

NaiveRearrangementRRT::~NaiveRearrangementRRT() = default;

void NaiveRearrangementRRT::setup(PlanningQueryPtr pq, PlanningBlackboard& blackboard)
{
    _control_sampler.setK(_num_control_samples);
    RearrangementRRT::setup(pq, blackboard);
}

RearrangementPlanner::PlanningQueryPtr NaiveRearrangementRRT::createPlanningQuery(
    mps_state::goal::ObjectsRelocationGoalPtr goal_region,
    mps::planner::ompl::state::SimEnvWorldState* start_state,
    const std::string& robot_name,
    float timeout)
{
    auto pq = RearrangementRRT::createPlanningQuery(goal_region, start_state, robot_name, timeout);
    using std::placeholders::_1;
    // declare num control samples
    std::function<void(unsigned int)> cs_setter = std::bind(&NaiveRearrangementRRT::setNumControlSamples, this, _1);
    std::function<unsigned int()> cs_getter = std::bind(&NaiveRearrangementRRT::getNumControlSamples, this);
    pq->parameters->declareParam<unsigned int>("num_control_samples", cs_setter, cs_getter);
    pq->parameters->setParam("num_control_samples", "10");
    return pq;
}

unsigned int NaiveRearrangementRRT::getNumControlSamples() const
{
    return _num_control_samples;
}

void NaiveRearrangementRRT::setNumControlSamples(unsigned int num_samples)
{
    _num_control_samples = num_samples;
}

bool NaiveRearrangementRRT::extend(mps::planner::ompl::planning::essentials::MotionPtr start,
    ::ompl::base::State* dest,
    unsigned int active_obj_id,
    mps::planner::ompl::planning::essentials::MotionPtr& last_motion,
    PlanningBlackboard& pb)
{
    static const std::string log_prefix("[mps::planner::pushing::algorithm::NaiveRearrangementRRT::extend]");
    MotionPtr new_motion = _motion_cache.getNewMotion();
    _si->copyState(new_motion->getState(), dest);
    // sampleTo(c, is, ns) - samples a control c that moves is towards ns.
    // ns is overwritten to be the actual outcome (i.e. ns = f(is, c))
    // the return value is either 0 if sampling a valid control failed
    logging::logDebug("Sampling a control", log_prefix);
    _distance_measure->setAll(false);
    _distance_measure->setActive(active_obj_id, true);
    last_motion = start;
    unsigned int num_steps = _control_sampler.sampleTo(new_motion->getControl(),
        start->getState(),
        new_motion->getState());
    bool reached_a_goal = pb.pq->goal_region->isSatisfied(new_motion->getState());
    if (num_steps > 0) { // the sampled control is valid, i.e. the outcoming state is valid
#ifdef DEBUG_PRINTOUTS
        printState("Extending towards state ", new_motion->getState());
#endif
        addToTree(new_motion, start, pb);
        last_motion = new_motion;
    } else {
        logging::logDebug("Could not find a valid control", log_prefix);
        _motion_cache.cacheMotion(new_motion);
    }
    pb.stats.num_state_propagations += _control_sampler.getK();
    return reached_a_goal;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// HybridActionRRT ////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
HybridActionRRT::HybridActionRRT(::ompl::control::SpaceInformationPtr si,
    mps::planner::pushing::oracle::PushingOraclePtr pushing_oracle,
    mps::planner::pushing::oracle::RobotOraclePtr robot_oracle,
    const std::string& robot_name)
    : RearrangementRRT(si)
{
    _oracle_sampler = std::make_shared<mps::planner::pushing::oracle::OracleControlSampler>(si, pushing_oracle, robot_oracle, robot_name);
    _state_propagator = std::dynamic_pointer_cast<mps_control::SimEnvStatePropagator>(si->getStatePropagator());
    assert(_state_propagator);
}

HybridActionRRT::~HybridActionRRT() = default;

RearrangementPlanner::PlanningQueryPtr HybridActionRRT::createPlanningQuery(
    mps_state::goal::ObjectsRelocationGoalPtr goal_region,
    mps::planner::ompl::state::SimEnvWorldState* start_state,
    const std::string& robot_name,
    float timeout)
{
    auto pq = RearrangementRRT::createPlanningQuery(goal_region, start_state, robot_name, timeout);
    using std::placeholders::_1;
    // declare num control samples
    std::function<void(unsigned int)> cs_setter = std::bind(&HybridActionRRT::setNumControlSamples, this, _1);
    std::function<unsigned int()> cs_getter = std::bind(&HybridActionRRT::getNumControlSamples, this);
    pq->parameters->declareParam<unsigned int>("num_control_samples", cs_setter, cs_getter);
    pq->parameters->setParam("num_control_samples", "10");
    // declare action randomness
    std::function<void(float)> ar_setter = std::bind(&HybridActionRRT::setActionRandomness, this, _1);
    std::function<float()> ar_getter = std::bind(&HybridActionRRT::getActionRandomness, this);
    pq->parameters->declareParam<float>("action_randomness", ar_setter, ar_getter);
    pq->parameters->setParam("action_randomness", "0.0001");
    return pq;
}

unsigned int HybridActionRRT::getNumControlSamples() const
{
    return _num_control_samples;
}

void HybridActionRRT::setNumControlSamples(unsigned int k)
{
    _num_control_samples = k;
}

float HybridActionRRT::getActionRandomness() const
{
    return _action_randomness;
}

void HybridActionRRT::setActionRandomness(float ar)
{
    _action_randomness = ar;
}

bool HybridActionRRT::extend(mps::planner::ompl::planning::essentials::MotionPtr start,
    ::ompl::base::State* dest,
    unsigned int active_obj_id,
    mps::planner::ompl::planning::essentials::MotionPtr& last_motion,
    PlanningBlackboard& pb)
{
    static const std::string log_prefix("[HybridActionRRT::extend]");
    float best_distance = std::numeric_limits<float>::max();
    std::vector<MotionPtr> best_state_action_sequence;
    for (unsigned int i = 0; i < _num_control_samples; ++i) {
        std::vector<::ompl::control::Control const*> controls;
        sampleActionSequence(controls, start, dest, pb);
        std::vector<MotionPtr> new_motion_sequence;
        forwardPropagateActionSequence(controls, start, new_motion_sequence, pb);
        if (not new_motion_sequence.empty()) {
            // compute the distance between the last state of this sequence and our destination
            auto& last_motion_candidate = new_motion_sequence[new_motion_sequence.size() - 1];
            float new_distance = _state_space->distance(dest, last_motion_candidate->getState());
            if (new_distance < best_distance) {
                // first free the best_state_sequence
                freeMotionList(best_state_action_sequence);
                best_state_action_sequence = new_motion_sequence;
                best_distance = new_distance;
            } else {
                // free the candidate sequence
                freeMotionList(new_motion_sequence);
            }
        }
    }
    // finally, we can add the best sequence to our tree
    auto prev_motion = start;
    for (auto& motion : best_state_action_sequence) {
        addToTree(motion, prev_motion, pb);
        prev_motion = motion;
        last_motion = motion;
        if (pb.pq->goal_region->isSatisfied(motion->getState())) {
            return true;
        }
    }
    return false;
}

void HybridActionRRT::sampleActionSequence(std::vector<::ompl::control::Control const*>& controls,
    mps::planner::ompl::planning::essentials::MotionPtr start,
    ::ompl::base::State* dest,
    PlanningBlackboard& pb)
{
    static const std::string log_prefix("[HybridActionRRT::sampleActionSequence]");
    float random_die = _rng->uniform01();
#ifdef DEBUG_PRINTOUTS
    printState(log_prefix + "Sampling action sequence given state ", start->getState());
#endif
    if (random_die < _action_randomness) {
        logging::logDebug("Sampling random action sequence", log_prefix);
        // sample random action
        _oracle_sampler->randomControl(controls);
    } else { // these are our primitives
        // first sample which object to move
        unsigned int obj_id = sampleActiveObject(pb);
        if (obj_id == pb.robot_id) { // steer robot, i.e. transit primitive
            logging::logDebug("Sampling transit action", log_prefix);
            _oracle_sampler->steerRobot(controls, start->getConstState(), dest);
        } else { // transfer primitive
            logging::logDebug("Sampling transfer action", log_prefix);
            auto new_motion = _motion_cache.getNewMotion();
            _state_space->copyState(new_motion->getState(), start->getState());
            // first sample robot state
            _oracle_sampler->sampleFeasibleState(new_motion->getState(), dest, obj_id);
#ifdef DEBUG_PRINTOUTS
            printState(log_prefix + " Steering robot first to feasible state ", new_motion->getState());
#endif
            // compute controls to move to that state
            _oracle_sampler->steerRobot(controls, start->getConstState(), new_motion->getState());
            // TODO we could now either forward propagate these controls, or we beam the robot to this state
            // forward propagating would make the primitive slower to compute, but a bit more capable
            // for now we beam it only, i.e. we assume the world state didn't change by the previous movement
            _oracle_sampler->steerPush(controls, new_motion->getState(), dest, obj_id);
            _motion_cache.cacheMotion(new_motion);
        }
    }
}

void HybridActionRRT::forwardPropagateActionSequence(const std::vector<::ompl::control::Control const*>& controls,
    mps::planner::ompl::planning::essentials::MotionPtr start,
    std::vector<MotionPtr>& state_action_seq,
    PlanningBlackboard& pb)
{
    static const std::string log_prefix("[HybridActionRRT::forwardPropagateActionSequence]");
    // now forward propagate these controls
    bool extension_success = false;
    auto prev_motion = start;
#ifdef DEBUG_PRINTOUTS
    printState(log_prefix + " Starting forward propagation from state ", start->getState());
#endif
    for (auto const* control : controls) {
        MotionPtr new_motion = _motion_cache.getNewMotion();
        _si->copyControl(new_motion->getControl(), control);
        extension_success = _state_propagator->propagate(prev_motion->getState(),
            new_motion->getControl(),
            new_motion->getState());
#ifdef DEBUG_PRINTOUTS
        printState(log_prefix + " Propagated to state ", new_motion->getState());
#endif
        pb.stats.num_state_propagations++;
        if (not extension_success) { // this action primitive ends here
            logging::logDebug("An action sequence failed, aborting forward propagation", log_prefix);
            _motion_cache.cacheMotion(new_motion);
            break;
        }
#ifdef DEBUG_PRINTOUTS
        printState("Oracle control took us to state ", new_motion->getState());
#endif
        state_action_seq.push_back(new_motion);
        // otherwise we just continue extending as long as we have controls
        prev_motion = new_motion;
    }
}

void HybridActionRRT::freeMotionList(std::vector<MotionPtr>& motions)
{
    for (auto& motion : motions) {
        _motion_cache.cacheMotion(motion);
    }
    motions.clear();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// OracleRearrangementRRT /////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
OracleRearrangementRRT::OracleRearrangementRRT(::ompl::control::SpaceInformationPtr si,
    mps::planner::pushing::oracle::PushingOraclePtr pushing_oracle,
    mps::planner::pushing::oracle::RobotOraclePtr robot_oracle,
    const std::string& robot_name)
    : RearrangementRRT(si)
{
    _oracle_sampler = std::make_shared<mps::planner::pushing::oracle::OracleControlSampler>(si, pushing_oracle, robot_oracle, robot_name);
    _state_propagator = std::dynamic_pointer_cast<mps_control::SimEnvStatePropagator>(si->getStatePropagator());
    assert(_state_propagator);
}

OracleRearrangementRRT::~OracleRearrangementRRT() = default;

RearrangementPlanner::PlanningQueryPtr OracleRearrangementRRT::createPlanningQuery(
    mps_state::goal::ObjectsRelocationGoalPtr goal_region,
    mps::planner::ompl::state::SimEnvWorldState* start_state,
    const std::string& robot_name,
    float timeout)
{
    auto pq = RearrangementRRT::createPlanningQuery(goal_region, start_state, robot_name, timeout);
    using std::placeholders::_1;
    // declare action randomness
    std::function<void(float)> ar_setter = std::bind(&OracleRearrangementRRT::setActionRandomness, this, _1);
    std::function<float()> ar_getter = std::bind(&OracleRearrangementRRT::getActionRandomness, this);
    pq->parameters->declareParam<float>("action_randomness", ar_setter, ar_getter);
    pq->parameters->setParam("action_randomness", "0.0001");
    // declare feasible state noise
    std::function<void(float)> sn_setter = std::bind(&OracleRearrangementRRT::setStateNoise, this, _1);
    std::function<float()> sn_getter = std::bind(&OracleRearrangementRRT::getStateNoise, this);
    pq->parameters->declareParam<float>("state_noise", sn_setter, sn_getter);
    pq->parameters->setParam("state_noise", "0.00001");
    return pq;
}

float OracleRearrangementRRT::getActionRandomness() const
{
    return _action_randomness;
}

void OracleRearrangementRRT::setActionRandomness(float ar)
{
    _action_randomness = ar;
}

float OracleRearrangementRRT::getStateNoise() const
{
    return _feasible_state_noise;
}

void OracleRearrangementRRT::setStateNoise(float sn)
{
    _feasible_state_noise = sn;
}

void OracleRearrangementRRT::selectTreeNode(const ompl::planning::essentials::MotionPtr& sample_motion,
    ompl::planning::essentials::MotionPtr& selected_node,
    unsigned int& active_obj_id,
    bool sample_is_goal,
    PlanningBlackboard& pb)
{
    static const std::string log_prefix("[mps::planner::pushing::algorithm::OracleRearrangementRRT::selectTreeNode]");
    // TODO we only overwrite this because we need to sample a feasible state.
    RearrangementRRT::selectTreeNode(sample_motion, selected_node, active_obj_id, sample_is_goal, pb);
    // now sample a feasible state for pushing
    // TODO This is a bit of a hack and should probably be done in a cleaner way
    // TODO semantically this is part of extend and not selectTreeNode. The reason it is here is in SliceBasedOracleRRT
    if (active_obj_id != pb.robot_id) {
        // sample a feasible robot state for the desired push
        auto tmp_motion = _motion_cache.getNewMotion();
        // tmp_motion = slice_representative
        _state_space->copyState(tmp_motion->getState(), selected_node->getState());
        // tmp_motion's robot state gets updated with feasible robot state
        _oracle_sampler->sampleFeasibleState(tmp_motion->getState(), sample_motion->getState(), active_obj_id, _feasible_state_noise);
// save that robot state in sample_motion
#ifdef DEBUG_PRINTOUTS
        printState(log_prefix + " For sample ", sample_motion->getState());
#endif
        _state_space->copySubState(sample_motion->getState(), tmp_motion->getState(), pb.robot_id);
#ifdef DEBUG_PRINTOUTS
        printState(log_prefix + " Sampled feasible state ", sample_motion->getState());
#endif
        _motion_cache.cacheMotion(tmp_motion);
    }
}

bool OracleRearrangementRRT::extend(mps::planner::ompl::planning::essentials::MotionPtr start,
    ::ompl::base::State* dest, unsigned int active_obj_id,
    mps::planner::ompl::planning::essentials::MotionPtr& last_motion,
    PlanningBlackboard& pb)
{
    static const std::string log_prefix("[mps::planner::pushing::algorithm::OracleRearrangementRRT::extend]");
    logging::logDebug("Attempting to extend search tree", log_prefix);
    std::vector<const ::ompl::control::Control*> controls;
    bool b_goal = false;
    bool extension_success = false;
    float random_die = _rng->uniform01();
    if (random_die < _action_randomness) {
        logging::logDebug("Sampling random action sequence", log_prefix);
        // sample random action
        _oracle_sampler->randomControl(controls);
        extendStep(controls, start, last_motion, pb, extension_success, b_goal);
    } else { // these are our primitives
        // first only move the robot TODO: This often pushes the object away from us
        _oracle_sampler->steerRobot(controls, start->getState(), dest);
        if (controls.empty()) {
            logging::logErr("OracleControlSampler provided no controls at all", log_prefix);
            return false;
        }
        extendStep(controls, start, last_motion, pb, extension_success, b_goal);
        // next, if the active object is not the robot, try a push
        if (extension_success and active_obj_id != pb.robot_id and not b_goal) {
            logging::logDebug("Steering robot to feasible state successful, attempting push", log_prefix);
            controls.clear();
            _oracle_sampler->steerPush(controls, last_motion->getState(), dest, active_obj_id);
            extendStep(controls, last_motion, last_motion, pb, extension_success, b_goal);
        }
    }
    return b_goal;
}

void OracleRearrangementRRT::extendStep(const std::vector<const ::ompl::control::Control*>& controls,
    const mps::planner::ompl::planning::essentials::MotionPtr& start_motion,
    mps::planner::ompl::planning::essentials::MotionPtr& result_motion,
    PlanningBlackboard& pb,
    bool& extension_success,
    bool& goal_reached)
{
    static const std::string log_prefix("[mps::planner::pushing::algorithm::OracleRearrangementRRT::extendStep]");
    goal_reached = false;
    extension_success = false;
    auto prev_motion = start_motion;
    for (auto const* control : controls) {
        MotionPtr new_motion = _motion_cache.getNewMotion();
        _si->copyControl(new_motion->getControl(), control);
        extension_success = _state_propagator->propagate(prev_motion->getState(),
            new_motion->getControl(),
            new_motion->getState());
        pb.stats.num_state_propagations++;
        if (not extension_success) { // we failed, no tree extension
            logging::logDebug("A control provided by the oracle failed. ", log_prefix);
            _motion_cache.cacheMotion(new_motion);
            return;
        }
#ifdef DEBUG_PRINTOUTS
        printState("Oracle control took us to state ", new_motion->getState());
#endif
        // we extended the tree a bit, add this new state to the tree
        addToTree(new_motion, prev_motion, pb);
        result_motion = new_motion;
        if (pb.pq->goal_region->isSatisfied(new_motion->getState())) {
            // we reached a goal!
            goal_reached = true;
            return;
        }
        // otherwise we just continue extending as long as we have controls
        prev_motion = new_motion;
    }
}

mps::planner::pushing::oracle::OracleControlSamplerPtr OracleRearrangementRRT::getOracleSampler() const
{
    return _oracle_sampler;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// SliceBasedOracleRRT ///////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////// SliceBasedOracleRRT ////////////////////////////////////////////////
SliceBasedOracleRRT::SliceBasedOracleRRT(::ompl::control::SpaceInformationPtr si,
    mps::planner::pushing::oracle::PushingOraclePtr pushing_oracle,
    mps::planner::pushing::oracle::RobotOraclePtr robot_oracle,
    const std::string& robot_name)
    : OracleRearrangementRRT(si, pushing_oracle, robot_oracle, robot_name)
    , _robot_state_dist_fn(std::make_shared<RobotStateDistanceFn>(_state_space))
    , _slice_distance_fn(std::make_shared<ObjectArrangementDistanceFn>(_state_space))
    , _pushing_oracle(pushing_oracle)
    , _min_slice_distance(0.0)
    , _slice_cache(_robot_state_dist_fn)
{
    _slices_nn = std::make_shared<::ompl::NearestNeighborsGNAT<SlicePtr>>();
    _slices_nn->setDistanceFunction(std::bind(&ObjectArrangementDistanceFn::distance,
        std::ref(_slice_distance_fn),
        std::placeholders::_1,
        std::placeholders::_2));
}

SliceBasedOracleRRT::~SliceBasedOracleRRT() = default;

void SliceBasedOracleRRT::setup(PlanningQueryPtr pq, PlanningBlackboard& pb)
{
    RearrangementRRT::setup(pq, pb);
    _robot_state_space = std::dynamic_pointer_cast<mps_state::SimEnvObjectStateSpace>(_state_space->getSubspace(pb.robot_id));
    _robot_state_sampler = _robot_state_space->allocStateSampler();
    _slice_distance_fn->distance_measure.setWeights(pq->weights);
    _slice_distance_fn->setRobotId(pb.robot_id);
    _robot_state_dist_fn->distance_measure.setWeights(pq->weights);
    _robot_state_dist_fn->setRobotId(pb.robot_id);
    _slices_nn->clear();
    _slices_list.clear();
    _pushing_oracle->timer = timer_ptr;
    // set min slice distance
    // TODO should update this using state space and distance weights
    _min_slice_distance = (_state_space->getNumObjects() - 1) * 0.001;
}

void SliceBasedOracleRRT::selectTreeNode(const ompl::planning::essentials::MotionPtr& sample,
    ompl::planning::essentials::MotionPtr& selected_node,
    unsigned int& active_obj_id, bool sample_is_goal,
    PlanningBlackboard& pb)
{
    static const std::string log_prefix("[mps::planner::pushing::algorithm::SliceBasedOracleRRT]");
    logging::logDebug("Selecting tree node to extend for given sample", log_prefix);
    // pick the slice that is closest to the sample
    SlicePtr nearest_slice = getSlice(sample, pb);
#ifdef DEBUG_PRINTOUTS
    printState("Rerpresentative of nearest slice is ", nearest_slice->repr->getState());
#endif

    if (active_obj_id == pb.robot_id) {
        // the selected node is the nearest node to the sample within the selected slice (in terms of robot distance)
        selected_node = nearest_slice->slice_samples_nn->nearest(sample);
    } else {
#ifdef DEBUG_PRINTOUTS
        printState("Sample is ", sample->getState());
#endif
        auto selected_slice = nearest_slice;

#ifdef DEBUG_PRINTOUTS
        printState("Slice selected for extension: ", selected_slice->repr->getState());
#endif
        // sample a feasible robot state for the desired push
        auto tmp_motion = _motion_cache.getNewMotion();
        // tmp_motion = slice_representative
        _state_space->copyState(tmp_motion->getState(), selected_slice->repr->getState());
#ifdef DEBUG_PRINTOUTS
        printState("Sample to move to is ", sample->getState());
#endif
        // tmp_motion's robot state gets updated with feasible robot state
        _oracle_sampler->sampleFeasibleState(tmp_motion->getState(), sample->getState(), active_obj_id,
            _feasible_state_noise);
#ifdef DEBUG_PRINTOUTS
        printState("Sampled feasible state is ", tmp_motion->getState());
#endif
        // save that robot state in sample
        _state_space->copySubState(sample->getState(), tmp_motion->getState(), pb.robot_id);
        _motion_cache.cacheMotion(tmp_motion);
        // sample the node that is closest to a feasible state in this slice
        selected_node = selected_slice->slice_samples_nn->nearest(sample);
    }
}

void SliceBasedOracleRRT::addToTree(MotionPtr new_motion, MotionPtr parent, PlanningBlackboard& pb)
{
    RearrangementRRT::addToTree(new_motion, parent, pb);
    SlicePtr closest_slice = getSlice(new_motion, pb);
    float slice_distance = distanceToSlice(new_motion, closest_slice);
    if (slice_distance > _min_slice_distance) { // we discovered a new slice!
        auto new_slice = _slice_cache.getNewSlice(new_motion);
        _slices_nn->add(new_slice);
        _slices_list.push_back(new_slice);
        if (_debug_drawer) {
            _debug_drawer->addNewSlice(new_slice);
        }
    } else { // the new motion/state is in the same slice
        closest_slice->addSample(new_motion);
    }
}

SlicePtr SliceBasedOracleRRT::getSlice(MotionPtr motion, PlanningBlackboard& pb) const
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

float SliceBasedOracleRRT::distanceToSlice(ompl::planning::essentials::MotionPtr motion, SlicePtr slice) const
{
    if (!slice) {
        return std::numeric_limits<float>::max();
    }
    auto query_slice = _slice_cache.getNewSlice(motion);
    float distance = (float)_slice_distance_fn->distance(query_slice, slice);
    _slice_cache.cacheSlice(query_slice);
    return distance;
}

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
        _oracle_sampler->sampleFeasibleState(feasible_state_mtn->getState(),
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
        _oracle_sampler->sampleFeasibleState(feasible_state_mtn->getState(),
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
