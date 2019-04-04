//
// Created by joshua on 8/14/17.
//

#include <mps/planner/ompl/control/Interfaces.h>
#include <mps/planner/pushing/algorithm/SingleExtendRRT.h>
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
/////////////////////////////////////// SingleExtendRRT /////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
SingleExtendRRT::SingleExtendRRT(::ompl::control::SpaceInformationPtr si)
    : RearrangementPlanner(si)
    , _motion_cache(si)
    , _log_prefix("[mps::planner::pushing::algorithm::SingleExtendRRT::")
{
    _state_space = std::dynamic_pointer_cast<mps_state::SimEnvWorldStateSpace>(_si->getStateSpace());
    if (!_state_space) {
        throw std::logic_error(_log_prefix + "setup] Could not cast state space to SimEnvWorldStateSpace");
    }
    _state_propagator = std::dynamic_pointer_cast<mps_control::SimEnvStatePropagator>(si->getStatePropagator());
    if (!_state_propagator) {
        throw std::logic_error(_log_prefix + "setup] Could not cast state propagator to SimEnvWorldStatePropagator");
    }
    _distance_measure = std::make_shared<mps::planner::ompl::state::SimEnvWorldStateDistanceMeasure>(_state_space);
    _state_space->setDistanceMeasure(_distance_measure);
    // set up our search tree
    if (!_tree) {
        _tree = std::make_shared<::ompl::NearestNeighborsGNAT<mps::planner::ompl::planning::essentials::MotionPtr>>();
        // _tree = std::make_shared<::ompl::NearestNeighborsSqrtApprox <mps::planner::ompl::planning::essentials::MotionPtr> >();
        using namespace std::placeholders;
        _tree->setDistanceFunction(std::bind(&SingleExtendRRT::treeDistanceFunction, this, _1, _2));
    }
    // _state_sampler = _si->allocStateSampler();
    _state_sampler = _si->allocValidStateSampler();
    assert(_state_space->getNumObjects() > 1);
    _rng = mps::planner::util::random::getDefaultRandomGenerator();
}

SingleExtendRRT::~SingleExtendRRT() = default;

void SingleExtendRRT::setup(PlanningQueryPtr pq, PlanningBlackboard& blackboard)
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

bool SingleExtendRRT::plan(PlanningQueryPtr pq, PlanningStatistics& stats)
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
    _timer->startTimer(pq->time_out);
    // Do the actual planning
    while (not _timer->timeOutExceeded() and not pq->stopping_condition() && !solved) {
        blackboard.stats.num_iterations++;
        // sample a new state
        unsigned int active_obj_id = 0;
        sample(sample_motion, active_obj_id, blackboard);
#ifdef DEBUG_PRINTOUTS
        printState("Sampled state is ", sample_motion->getState()); // TODO remove
#endif
        // Get a tree node to expand
        selectTreeNode(sample_motion, current_motion, active_obj_id, blackboard);
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

    blackboard.stats.runtime = _timer->stopTimer();
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

RearrangementPlanner::PlanningQueryPtr SingleExtendRRT::createPlanningQuery(
    mps_state::goal::ObjectsRelocationGoalPtr goal_region,
    mps::planner::ompl::state::SimEnvWorldState* start_state,
    const std::string& robot_name,
    float timeout)
{
    auto pq = RearrangementPlanner::createPlanningQuery(goal_region, start_state, robot_name, timeout);
    using std::placeholders::_1;
    // declare goal bias
    std::function<void(float)> gb_setter = std::bind(&SingleExtendRRT::setGoalBias, this, _1);
    std::function<float()> gb_getter = std::bind(&SingleExtendRRT::getGoalBias, this);
    pq->parameters->declareParam<float>("goal_bias", gb_setter, gb_getter);
    pq->parameters->setParam("goal_bias", "0.2");
    // declare target bias
    std::function<void(float)> tb_setter = std::bind(&SingleExtendRRT::setTargetBias, this, _1);
    std::function<float()> tb_getter = std::bind(&SingleExtendRRT::getTargetBias, this);
    pq->parameters->declareParam<float>("target_bias", tb_setter, tb_getter);
    pq->parameters->setParam("target_bias", "0.25");
    // declare robot bias
    std::function<void(float)> rb_setter = std::bind(&SingleExtendRRT::setRobotBias, this, _1);
    std::function<float()> rb_getter = std::bind(&SingleExtendRRT::getRobotBias, this);
    pq->parameters->declareParam<float>("robot_bias", rb_setter, rb_getter);
    pq->parameters->setParam("robot_bias", "0.0");
    return pq;
}

bool SingleExtendRRT::isGoalPath(PathConstPtr path, const mps_state::SimEnvWorldState* start, PlanningQueryPtr pq,
    PathPtr update_path)
{
    if (!_si->isValid(start))
        return false;
    auto motion = std::make_shared<Motion>(_si);
    auto prev_motion = motion;
    _si->copyState(motion->getState(), start);
    _si->nullControl(motion->getControl());
    auto robot_state_space = _state_space->getObjectStateSpace(pq->robot_name);
    unsigned int robot_id = _state_space->getSubspaceIndex(pq->robot_name);
    bool valid_path = true;
    for (unsigned int i = 0; i < path->getNumMotions(); ++i) {
        auto pathm = path->getConstMotion(i);
        // propagate action
        _si->copyControl(motion->getControl(), pathm->getConstControl());
        valid_path = _state_propagator->propagate(prev_motion->getState(), motion->getControl(), motion->getState());
        if (!valid_path)
            break;
        if (update_path) {
            update_path->append(motion);
            prev_motion = motion;
            motion = std::make_shared<Motion>(_si);
        }
    }
    bool goal_reached = pq->goal_region->isSatisfied(motion->getState());
    return goal_reached and valid_path;
}

bool SingleExtendRRT::sample(mps::planner::ompl::planning::essentials::MotionPtr motion,
    unsigned int& target_obj_id,
    PlanningBlackboard& pb)
{
    static const std::string log_prefix("mps::planner::pushing::algorithm::SingleExtendRRT::sample]");
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

void SingleExtendRRT::selectTreeNode(const ompl::planning::essentials::MotionPtr& sample_motion,
    ompl::planning::essentials::MotionPtr& selected_node,
    unsigned int& active_obj_id,
    PlanningBlackboard& pb)
{
    static const std::string log_prefix("[mps::planner::pushing::algorithm::SingleExtendRRT::selectTreeNode]");
    logging::logDebug("Searching for nearest neighbor.", log_prefix);
    // select nearest state using full state space distance function
    _distance_measure->setAll(true); // we take the full state into account here
    selected_node = _tree->nearest(sample_motion);
    pb.stats.num_nearest_neighbor_queries++;
}

float SingleExtendRRT::getGoalBias() const
{
    return _goal_bias;
}
float SingleExtendRRT::getTargetBias() const
{
    return _target_bias;
}
float SingleExtendRRT::getRobotBias() const
{
    return _robot_bias;
}

void SingleExtendRRT::setGoalBias(float v)
{
    _goal_bias = v;
}

void SingleExtendRRT::setTargetBias(float v)
{
    _target_bias = v;
}

void SingleExtendRRT::setRobotBias(float v)
{
    _robot_bias = v;
}

void SingleExtendRRT::addToTree(mps::planner::ompl::planning::essentials::MotionPtr new_motion,
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

unsigned int SingleExtendRRT::sampleActiveObject(const PlanningBlackboard& pb) const
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

double SingleExtendRRT::treeDistanceFunction(const MotionPtr& a, const MotionPtr& b) const
{
    auto* state_a = a->getState();
    auto* state_b = b->getState();
    return _state_space->distance(state_a, state_b); // this uses _distance_measure
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// NaiveRearrangementRRT /////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
NaiveRearrangementRRT::NaiveRearrangementRRT(::ompl::control::SpaceInformationPtr si,
    unsigned int k)
    : SingleExtendRRT(si)
    , _control_sampler(si.get(), k)
{
}

NaiveRearrangementRRT::~NaiveRearrangementRRT() = default;

void NaiveRearrangementRRT::setup(PlanningQueryPtr pq, PlanningBlackboard& blackboard)
{
    _control_sampler.setK(_num_control_samples);
    SingleExtendRRT::setup(pq, blackboard);
}

RearrangementPlanner::PlanningQueryPtr NaiveRearrangementRRT::createPlanningQuery(
    mps_state::goal::ObjectsRelocationGoalPtr goal_region,
    mps::planner::ompl::state::SimEnvWorldState* start_state,
    const std::string& robot_name,
    float timeout)
{
    auto pq = SingleExtendRRT::createPlanningQuery(goal_region, start_state, robot_name, timeout);
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
    : SingleExtendRRT(si)
{
    _oracle_sampler = std::make_shared<mps::planner::pushing::oracle::OracleControlSampler>(si, pushing_oracle, robot_oracle, robot_name);
    assert(_state_propagator);
}

HybridActionRRT::~HybridActionRRT() = default;

RearrangementPlanner::PlanningQueryPtr HybridActionRRT::createPlanningQuery(
    mps_state::goal::ObjectsRelocationGoalPtr goal_region,
    mps::planner::ompl::state::SimEnvWorldState* start_state,
    const std::string& robot_name,
    float timeout)
{
    auto pq = SingleExtendRRT::createPlanningQuery(goal_region, start_state, robot_name, timeout);
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
    : SingleExtendRRT(si)
{
    _oracle_sampler = std::make_shared<mps::planner::pushing::oracle::OracleControlSampler>(si, pushing_oracle, robot_oracle, robot_name);
}

OracleRearrangementRRT::~OracleRearrangementRRT() = default;

RearrangementPlanner::PlanningQueryPtr OracleRearrangementRRT::createPlanningQuery(
    mps_state::goal::ObjectsRelocationGoalPtr goal_region,
    mps::planner::ompl::state::SimEnvWorldState* start_state,
    const std::string& robot_name,
    float timeout)
{
    auto pq = SingleExtendRRT::createPlanningQuery(goal_region, start_state, robot_name, timeout);
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
    PlanningBlackboard& pb)
{
    static const std::string log_prefix("[mps::planner::pushing::algorithm::OracleRearrangementRRT::selectTreeNode]");
    // TODO we only overwrite this because we need to sample a feasible state.
    SingleExtendRRT::selectTreeNode(sample_motion, selected_node, active_obj_id, pb);
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
    SingleExtendRRT::setup(pq, pb);
    _robot_state_space = std::dynamic_pointer_cast<mps_state::SimEnvObjectStateSpace>(_state_space->getSubspace(pb.robot_id));
    _robot_state_sampler = _robot_state_space->allocStateSampler();
    _slice_distance_fn->distance_measure.setWeights(pq->weights);
    _slice_distance_fn->setRobotId(pb.robot_id);
    _robot_state_dist_fn->setRobotId(pb.robot_id);
    _slices_nn->clear();
    _pushing_oracle->timer = _timer; // give pushing oracle access to our timer, so it can add its computation time (in a different process)
    // set min slice distance
    // TODO should update this using state space and distance weights
    _min_slice_distance = (_state_space->getNumObjects() - 1) * 0.001;
}

void SliceBasedOracleRRT::selectTreeNode(const ompl::planning::essentials::MotionPtr& sample,
    ompl::planning::essentials::MotionPtr& selected_node,
    unsigned int& active_obj_id, PlanningBlackboard& pb)
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
    SingleExtendRRT::addToTree(new_motion, parent, pb);
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
