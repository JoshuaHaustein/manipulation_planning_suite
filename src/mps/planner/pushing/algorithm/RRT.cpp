//
// Created by joshua on 8/14/17.
//

#include <mps/planner/pushing/algorithm/RRT.h>
#include <mps/planner/util/Logging.h>
#include <mps/planner/ompl/control/Interfaces.h>
#define GNAT_SAMPLER
#include <ompl/datastructures/NearestNeighborsGNAT.h>
#include <ompl/datastructures/NearestNeighborsSqrtApprox.h>

#include <queue>
#include <boost/functional/hash.hpp>

namespace logging = mps::planner::util::logging;
namespace ob = ::ompl::base;
namespace oc = ::ompl::control;
namespace mps_state = mps::planner::ompl::state;
namespace mps_control = mps::planner::ompl::control;
using namespace mps::planner::pushing::algorithm;
using namespace mps::planner::ompl::planning::essentials;

////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////// PlanningQuery ///////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
RearrangementRRT::PlanningQuery::PlanningQuery(ompl::state::goal::ObjectsRelocationGoalPtr goal_region,
                                             ob::State *start_state,
                                             float time_out,
                                             const std::string& robot_name) :
    goal_region(goal_region),
    start_state(start_state),
    robot_name(robot_name),
    time_out(time_out)
{
    stopping_condition = []() {return false;};
    goal_bias = 0.2f;
    robot_bias = 0.0f;
    target_bias = 0.25f;
    min_state_distance = 0.001;
    min_slice_distance = min_state_distance;
    do_slice_ball_projection = true;
    max_slice_distance = 0.0;
    action_randomness = 0.0001f;
    feasible_state_noise = 0.00001f;
    num_control_samples = 10;
}

RearrangementRRT::PlanningQuery::PlanningQuery(const PlanningQuery &other) = default;

std::string RearrangementRRT::PlanningQuery::toString() const {
    std::stringstream ss;
    // TODO for completeness should also print target and start state
    ss << "Robot name: " << robot_name << "\n";
    ss << "timeout: " << time_out << "\n";
    ss << "goal_bias: " << goal_bias << "\n";
    ss << "target_bias: " << target_bias << "\n";
    ss << "robot_bias: " << robot_bias << "\n";
    ss << "min_state_distance: " << min_state_distance << "\n";
    ss << "max_slice_distance: " << max_slice_distance << "\n";
    ss << "do_slice_ball_projection: " << do_slice_ball_projection << "\n";
    ss << "action_randomness: " << action_randomness << "\n";
    ss << "feasible state noise: " << feasible_state_noise << "\n";
    ss << "num_control_samples: " << num_control_samples << "\n";
    ss << "distance weights: ";
    for (auto& weight : weights) {
        ss << weight << ", ";
    }
    ss << "\n";
    return ss.str();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// PlanningBlackboard ////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
RearrangementRRT::PlanningBlackboard::PlanningBlackboard(PlanningQuery pq) :
        pq(pq),
        robot_id(0)
{
}


////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////// RearrangementRRT /////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
RearrangementRRT::RearrangementRRT(::ompl::control::SpaceInformationPtr si) :
        _si(si),
        _log_prefix("[mps::planner::pushing::algorithm::RearrangementRRT::")
{
    _state_space = std::dynamic_pointer_cast<mps_state::SimEnvWorldStateSpace>(_si->getStateSpace());
    if (!_state_space) {
        throw std::logic_error(_log_prefix + "setup] Could not cast state space to SimEnvWorldStateSpace");
    }
    _distance_measure = std::make_shared<mps::planner::ompl::state::SimEnvWorldStateDistanceMeasure>(_state_space);
    _state_space->setDistanceMeasure(_distance_measure);
    // set up our search tree
    if (!_tree) {
       _tree = std::make_shared<::ompl::NearestNeighborsGNAT <mps::planner::ompl::planning::essentials::MotionPtr> >();
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

void RearrangementRRT::setup(const PlanningQuery& pq, PlanningBlackboard& blackboard) {
    static const std::string log_prefix(_log_prefix + "setup]");
    setupBlackboard(blackboard);
    _distance_measure->setAll(true);
    _distance_measure->setWeights(pq.weights);
    if (_debug_drawer) {
        _debug_drawer->clear();
    }
    _tree->clear();
    logging::logInfo("Planner setup for the following query:\n " + blackboard.pq.toString(), log_prefix);
}

bool RearrangementRRT::plan(const PlanningQuery& pq, mps::planner::ompl::planning::essentials::PathPtr path) {
    PlanningStatistics stats;
    return plan(pq, path, stats);
}

bool RearrangementRRT::plan(const PlanningQuery& pq,
                            PathPtr path,
                            PlanningStatistics& stats) {
    static const std::string log_prefix(_log_prefix + "plan]");
    PlanningBlackboard blackboard(pq);
    setup(pq, blackboard);
    logging::logDebug("Starting to plan", log_prefix);

    MotionPtr current_motion = getNewMotion();
    MotionPtr sample_motion = getNewMotion();
    MotionPtr final_motion = nullptr;
    // set the state to be the start state
    _si->copyState(current_motion->getState(), pq.start_state);
    // set the control to be null
    _si->nullControl(current_motion->getControl());
    // Initialize the tree
    addToTree(current_motion, nullptr, blackboard);
    final_motion = current_motion;

    bool solved = pq.goal_region->isSatisfied(current_motion->getState());
    std::stringstream ss;
    pq.goal_region->print(ss);
    logging::logDebug("Planning towards goal " + ss.str(), log_prefix);
    logging::logDebug("Entering main loop", log_prefix);
    timer_ptr->startTimer(pq.time_out);
    // Do the actual planning
    while(not timer_ptr->timeOutExceeded() and not pq.stopping_condition() && !solved) {
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
    if(solved){
        logging::logInfo("Found a solution", log_prefix);
        path->initBacktrackMotion(final_motion);
    }
    // clean up
    _tree->clear();

    logging::logInfo("Planning finished", log_prefix);
    stats = blackboard.stats;
    return solved;
}

bool RearrangementRRT::sample(mps::planner::ompl::planning::essentials::MotionPtr motion,
                              unsigned int& target_obj_id,
                              PlanningBlackboard& pb)
{
    static const std::string log_prefix("mps::planner::pushing::algorithm::RearrangementRRT::sample]");
    bool is_goal = false;
    // sample random state with goal biasing
    if( _rng->uniform01() < pb.pq.goal_bias && pb.pq.goal_region->canSample()){
        logging::logDebug("Sampling a goal state", log_prefix);
        pb.pq.goal_region->sampleGoal(motion->getState());
        target_obj_id = pb.pq.goal_region->sampleTargetObjectIndex();
        is_goal = true;
    }else{
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

unsigned int RearrangementRRT::sampleActiveObject(const PlanningBlackboard& pb) const {
    double random_value = _rng->uniform01();
    if (random_value < pb.pq.target_bias) {
        return pb.pq.goal_region->sampleTargetObjectIndex();
    } else if (random_value < pb.pq.target_bias + pb.pq.robot_bias) {
        return pb.robot_id;
    } else {
        int value = _rng->uniformInt(0, _state_space->getNumObjects() - 1);
        return static_cast<unsigned int>(value);
    }
}

void RearrangementRRT::printState(const std::string& msg, ::ompl::base::State *state) const {
    std::stringstream ss;
    auto* world_state = dynamic_cast<mps_state::SimEnvWorldState*>(state);
    ss.str("");
    ss << msg;
    world_state->print(ss);
    logging::logDebug(ss.str(), "[mps::planner::pushing::oracle::RearrangementRRT::printState]");
    #ifdef DEBUG_VISUALIZE
        if (_debug_drawer) _debug_drawer->showState(world_state, _state_space);
    #endif
}

MotionPtr RearrangementRRT::getNewMotion() {
    if (not _motions_cache.empty()) {
        MotionPtr ptr = _motions_cache.top();
        _motions_cache.pop();
        return ptr;
    }
    return std::make_shared<Motion>(_si);
}

void RearrangementRRT::cacheMotion(MotionPtr ptr) {
    _motions_cache.push(ptr);
}

double RearrangementRRT::treeDistanceFunction(const MotionPtr &a, const MotionPtr &b) const {
    auto* state_a = a->getState();
    auto* state_b = b->getState();
    return _state_space->distance(state_a, state_b); // this uses _distance_measure
}

void RearrangementRRT::setDebugDrawer(DebugDrawerPtr debug_drawer) {
    _debug_drawer = debug_drawer;
}

void RearrangementRRT::setupBlackboard(PlanningBlackboard &pb) {
    pb.robot_id = 0;
    {
        int tmp_robot_id = _state_space->getObjectIndex(pb.pq.robot_name);
        if (tmp_robot_id < 0) {
            throw std::logic_error("[mps::planner::pushing::oracle::RearrangementRRT::setupBlackboard]"
                    "Could not retrieve id for robot " + pb.pq.robot_name);
        }
        pb.robot_id = (unsigned int)tmp_robot_id;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// NaiveRearrangementRRT /////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
NaiveRearrangementRRT::NaiveRearrangementRRT(::ompl::control::SpaceInformationPtr si,
                                             unsigned int k) :
        RearrangementRRT(si),
        _control_sampler(si.get(), k)
{
}

NaiveRearrangementRRT::~NaiveRearrangementRRT() = default;

void NaiveRearrangementRRT::setup(const PlanningQuery& pq, PlanningBlackboard& blackboard) {
    _control_sampler.setK(pq.num_control_samples);
    RearrangementRRT::setup(pq, blackboard);
}

bool NaiveRearrangementRRT::extend(mps::planner::ompl::planning::essentials::MotionPtr start,
                                   ::ompl::base::State* dest,
                                   unsigned int active_obj_id,
                                   mps::planner::ompl::planning::essentials::MotionPtr& last_motion,
                                   PlanningBlackboard& pb)
{
    static const std::string log_prefix("[mps::planner::pushing::algorithm::NaiveRearrangementRRT::extend]");
    MotionPtr new_motion = getNewMotion();
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
    bool reached_a_goal = pb.pq.goal_region->isSatisfied(new_motion->getState());
    if (num_steps > 0) { // the sampled control is valid, i.e. the outcoming state is valid
        #ifdef DEBUG_PRINTOUTS
        printState("Extending towards state ", new_motion->getState());
        #endif
        addToTree(new_motion, start, pb);
        last_motion = new_motion;
    } else {
        logging::logDebug("Could not find a valid control", log_prefix);
        cacheMotion(new_motion);
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
                                 const std::string& robot_name) :
    RearrangementRRT(si)
{
    _oracle_sampler = std::make_shared<mps::planner::pushing::oracle::OracleControlSampler>(si, pushing_oracle, robot_oracle, robot_name);
    _state_propagator = std::dynamic_pointer_cast<mps_control::SimEnvStatePropagator>(si->getStatePropagator());
    assert(_state_propagator);
}

HybridActionRRT::~HybridActionRRT() = default;

bool HybridActionRRT::extend(mps::planner::ompl::planning::essentials::MotionPtr start,
                             ::ompl::base::State* dest,
                             unsigned int active_obj_id,
                             mps::planner::ompl::planning::essentials::MotionPtr& last_motion,
                             PlanningBlackboard& pb)
{
    static const std::string log_prefix("[HybridActionRRT::extend]");
    float best_distance = std::numeric_limits<float>::max();
    std::vector<MotionPtr> best_state_action_sequence;
    for (unsigned int i = 0; i < pb.pq.num_control_samples; ++i) {
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
        if (pb.pq.goal_region->isSatisfied(motion->getState())) {
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
    if (random_die < pb.pq.action_randomness) {
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
            auto new_motion = getNewMotion();
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
            cacheMotion(new_motion);
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
        MotionPtr new_motion = getNewMotion();
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
            cacheMotion(new_motion);
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

void HybridActionRRT::freeMotionList(std::vector<MotionPtr>& motions) {
    for (auto& motion : motions) {
        cacheMotion(motion);
    }
    motions.clear();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// OracleRearrangementRRT /////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
OracleRearrangementRRT::OracleRearrangementRRT(::ompl::control::SpaceInformationPtr si,
                                             mps::planner::pushing::oracle::PushingOraclePtr pushing_oracle,
                                             mps::planner::pushing::oracle::RobotOraclePtr robot_oracle,
                                             const std::string& robot_name) :
        RearrangementRRT(si)
{
    _oracle_sampler = std::make_shared<mps::planner::pushing::oracle::OracleControlSampler>(si, pushing_oracle, robot_oracle, robot_name);
    _state_propagator = std::dynamic_pointer_cast<mps_control::SimEnvStatePropagator>(si->getStatePropagator());
    assert(_state_propagator);
}

OracleRearrangementRRT::~OracleRearrangementRRT() = default;

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
        auto tmp_motion = getNewMotion();
        // tmp_motion = slice_representative
        _state_space->copyState(tmp_motion->getState(), selected_node->getState());
        // tmp_motion's robot state gets updated with feasible robot state
        _oracle_sampler->sampleFeasibleState(tmp_motion->getState(), sample_motion->getState(), active_obj_id, pb.pq.feasible_state_noise);
        // save that robot state in sample_motion
        #ifdef DEBUG_PRINTOUTS
        printState(log_prefix + " For sample ", sample_motion->getState());
        #endif
        _state_space->copySubState(sample_motion->getState(), tmp_motion->getState(), pb.robot_id);
        #ifdef DEBUG_PRINTOUTS
        printState(log_prefix + " Sampled feasible state ", sample_motion->getState());
        #endif
        cacheMotion(tmp_motion);
    }
}

bool OracleRearrangementRRT::extend(mps::planner::ompl::planning::essentials::MotionPtr start,
                                 ::ompl::base::State *dest, unsigned int active_obj_id,
                                 mps::planner::ompl::planning::essentials::MotionPtr &last_motion,
                                 PlanningBlackboard &pb)
{
    static const std::string log_prefix("[mps::planner::pushing::algorithm::OracleRearrangementRRT::extend]");
    logging::logDebug("Attempting to extend search tree", log_prefix);
    std::vector<const ::ompl::control::Control*> controls;
    bool b_goal = false;
    bool extension_success = false;
    float random_die = _rng->uniform01();
    if (random_die < pb.pq.action_randomness) {
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
                                        const mps::planner::ompl::planning::essentials::MotionPtr &start_motion,
                                        mps::planner::ompl::planning::essentials::MotionPtr &result_motion,
                                        PlanningBlackboard& pb,
                                        bool& extension_success,
                                        bool& goal_reached)
{
    static const std::string log_prefix("[mps::planner::pushing::algorithm::OracleRearrangementRRT::extendStep]");
    goal_reached = false;
    extension_success = false;
    auto prev_motion = start_motion;
    for (auto const* control : controls) {
        MotionPtr new_motion = getNewMotion();
        _si->copyControl(new_motion->getControl(), control);
        extension_success = _state_propagator->propagate(prev_motion->getState(),
                                                         new_motion->getControl(),
                                                         new_motion->getState());
        pb.stats.num_state_propagations++;
        if (not extension_success) { // we failed, no tree extension
            logging::logDebug("A control provided by the oracle failed. ", log_prefix);
            cacheMotion(new_motion);
            return;
        }
        #ifdef DEBUG_PRINTOUTS
        printState("Oracle control took us to state ", new_motion->getState());
        #endif
        // we extended the tree a bit, add this new state to the tree
        addToTree(new_motion, prev_motion, pb);
        result_motion = new_motion;
        if (pb.pq.goal_region->isSatisfied(new_motion->getState())) {
            // we reached a goal!
            goal_reached = true;
            return;
        }
        // otherwise we just continue extending as long as we have controls
        prev_motion = new_motion;
    }
}

mps::planner::pushing::oracle::OracleControlSamplerPtr OracleRearrangementRRT::getOracleSampler() const {
    return _oracle_sampler;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// SliceBasedOracleRRT ///////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////

//////////////////////////////////////// Slice /////////////////////////////////////////////////////
SliceBasedOracleRRT::Slice::Slice(mps::planner::ompl::planning::essentials::MotionPtr repr_in,
                                  SliceDistanceFn distance_fn)
{
    slice_samples_nn = std::make_shared<::ompl::NearestNeighborsGNAT<mps::planner::ompl::planning::essentials::MotionPtr> >();
    slice_samples_nn->setDistanceFunction(distance_fn);
    repr = repr_in;
    slice_samples_nn->add(repr);
    slice_samples_list.push_back(repr);
}

SliceBasedOracleRRT::Slice::~Slice() = default;

void SliceBasedOracleRRT::Slice::addSample(ompl::planning::essentials::MotionPtr motion) {
    slice_samples_list.push_back(motion);
    slice_samples_nn->add(motion);
}

void SliceBasedOracleRRT::Slice::clear() {
    repr = nullptr;
    slice_samples_nn->clear();
    slice_samples_list.clear();
}
void SliceBasedOracleRRT::Slice::reset(ompl::planning::essentials::MotionPtr repr) {
    clear();
    this->repr = repr;
    slice_samples_nn->add(repr);
    slice_samples_list.push_back(repr);
}

///////////////////////////////////// WithinSliceDistance ////////////////////////////////////////////
SliceBasedOracleRRT::WithinSliceDistance::WithinSliceDistance(ompl::state::SimEnvWorldStateSpacePtr state_space,
                                                              const std::vector<float> &weights) :
    distance_measure(state_space, weights)
{
}

void SliceBasedOracleRRT::WithinSliceDistance::setRobotId(unsigned int id) {
    distance_measure.setAll(false);
    distance_measure.setActive(id, true);
}

double SliceBasedOracleRRT::WithinSliceDistance::distance(const ompl::planning::essentials::MotionPtr &motion_a,
                                                          const ompl::planning::essentials::MotionPtr &motion_b) const
{
    return distance_measure.distance(motion_a->getState(), motion_b->getState());
}

///////////////////////////////////// SliceDistance ////////////////////////////////////////////
SliceBasedOracleRRT::SliceDistance::SliceDistance(ompl::state::SimEnvWorldStateSpacePtr state_space,
                                                  const std::vector<float> &weights) :
    distance_measure(state_space, weights)
{
}

void SliceBasedOracleRRT::SliceDistance::setRobotId(unsigned int id) {
    distance_measure.setAll(true);
    distance_measure.setActive(id, false);
}

double SliceBasedOracleRRT::SliceDistance::distance(const SliceConstPtr &slice_a, const SliceConstPtr &slice_b) const
{
    double dist = distance_measure.distance(slice_a->repr->getState(), slice_b->repr->getState());
    return dist;
}

//////////////////////////////////////// SliceBasedOracleRRT ////////////////////////////////////////////////
SliceBasedOracleRRT::SliceBasedOracleRRT(::ompl::control::SpaceInformationPtr si,
                                         mps::planner::pushing::oracle::PushingOraclePtr pushing_oracle,
                                         mps::planner::pushing::oracle::RobotOraclePtr robot_oracle,
                                         const std::string &robot_name) :
    OracleRearrangementRRT(si, pushing_oracle, robot_oracle, robot_name),
    _within_slice_distance_fn(_state_space),
    _slice_distance_fn(_state_space),
    _pushing_oracle(pushing_oracle)
{
    _slices_nn = std::make_shared<::ompl::NearestNeighborsGNAT <SlicePtr> >();
    _slices_nn->setDistanceFunction(std::bind(&SliceBasedOracleRRT::SliceDistance::distance,
                                              std::ref(_slice_distance_fn),
                                              std::placeholders::_1,
                                              std::placeholders::_2));
}

SliceBasedOracleRRT::~SliceBasedOracleRRT() = default;

void SliceBasedOracleRRT::setup(const PlanningQuery& pq, PlanningBlackboard& pb) {
    RearrangementRRT::setup(pq, pb);
    _robot_state_space = std::dynamic_pointer_cast<mps_state::SimEnvObjectStateSpace>(_state_space->getSubspace(pb.robot_id));
    _robot_state_sampler = _robot_state_space->allocStateSampler();
    _slice_distance_fn.distance_measure.setWeights(pq.weights);
    _slice_distance_fn.setRobotId(pb.robot_id);
    _within_slice_distance_fn.distance_measure.setWeights(pq.weights);
    _within_slice_distance_fn.setRobotId(pb.robot_id);
    _slices_nn->clear();
    _slices_list.clear();
    _pushing_oracle->timer = timer_ptr;
    if (pq.max_slice_distance <= 0.0f) {
        assert(_state_space->getNumObjects() > 1);
        pb.pq.max_slice_distance = (_state_space->getNumObjects() - 1) * _pushing_oracle->getMaximalPushingDistance();
    }
    // set min slice distance
    // TODO should update this using state space and distance weights
    pb.pq.min_slice_distance = (_state_space->getNumObjects() - 1) * pb.pq.min_state_distance;
    if (pb.pq.min_slice_distance >= pb.pq.max_slice_distance) {
        throw std::runtime_error("[SliceBasedOracleRRT::setup] min slice distance must be smaller max slice distance");
    }
}

void SliceBasedOracleRRT::selectTreeNode(const ompl::planning::essentials::MotionPtr &sample,
                                         ompl::planning::essentials::MotionPtr &selected_node,
                                         unsigned int &active_obj_id, bool sample_is_goal,
                                         PlanningBlackboard &pb)
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
        #ifdef USE_SLICE_BALL
            // check whether the closest slice is within max_slice_distance
            float slice_distance = distanceToSlice(sample, nearest_slice);
            // if (slice_distance > pb.pq.min_slice_distance) { // our sample lies within a new slice
            auto sample_slice = getNewSlice(sample);
            // get all neighbor slices within radius max_slice_distance
            std::vector<ExtensionCandidateTuple> candidate_states;
            std::vector<SlicePtr> candidate_slices;
            // check whether the sample slice is within a radius of max_slice_distance to the nearest slice
            if (pb.pq.do_slice_ball_projection and slice_distance > pb.pq.max_slice_distance) {
                // if not, project it
                logging::logDebug("Projecting sample slice to reachable slice ball", log_prefix);
                projectSliceOnBall(sample_slice, nearest_slice, pb.pq.max_slice_distance, pb);
                #ifdef DEBUG_PRINTOUTS
                printState("Projected slice is ", sample_slice->repr->getState());
                #endif
            }
            _slices_nn->nearestR(sample_slice, 1.00001f * pb.pq.max_slice_distance, candidate_slices);
            if (not pb.pq.do_slice_ball_projection and candidate_slices.empty()) {
                // in case we didn't project, there may be no neighbor within radius max_slice_distance
                logging::logDebug("Projection disabled, adding nearest slice to candidates", log_prefix);
                candidate_slices.push_back(nearest_slice);
            }
            assert(not candidate_slices.empty());
            // there is at least one slice we can extend the search from
            for (auto& candidate_slice : candidate_slices) {
                auto distance = _slice_distance_fn.distance(candidate_slice, sample_slice);
                // assert(distance > pb.pq.min_slice_distance);
                distance = std::max((float)distance, pb.pq.min_slice_distance); // if sample lies within a slice, prevent divison by zero
                auto weight = 1.0 / distance;
                // save what we found
                candidate_states.emplace_back(std::make_tuple(candidate_slice, weight));
            }
            // from all the slices we took a look at, pick one state
            auto selected_state_tuple = selectCandidateSlice(candidate_states);
            auto selected_slice = std::get<0>(selected_state_tuple);
        #else
            auto selected_slice = nearest_slice;
        #endif

        #ifdef DEBUG_PRINTOUTS
            printState("Slice selected for extension: ", selected_slice->repr->getState());
        #endif
        // sample a feasible robot state for the desired push
        auto tmp_motion = getNewMotion();
        // tmp_motion = slice_representative
        _state_space->copyState(tmp_motion->getState(), selected_slice->repr->getState());
        // tmp_motion's robot state gets updated with feasible robot state
        #ifdef DEBUG_PRINTOUTS
            printState("Sample to move to is ", sample->getState());
        #endif
        _oracle_sampler->sampleFeasibleState(tmp_motion->getState(), sample->getState(), active_obj_id,
                                                pb.pq.feasible_state_noise);
        // save that robot state in sample
        #ifdef DEBUG_PRINTOUTS
            printState("Sampled feasible state is ", tmp_motion->getState());
        #endif
        _state_space->copySubState(sample->getState(), tmp_motion->getState(), pb.robot_id);
        cacheMotion(tmp_motion);
        // sample the node that is closest to a feasible state in this slice
        selected_node = selected_slice->slice_samples_nn->nearest(sample);
        #ifdef USE_SLICE_BALL
            cacheSlice(sample_slice);
        #endif
        // }
    }
}

void SliceBasedOracleRRT::addToTree(MotionPtr new_motion, MotionPtr parent, PlanningBlackboard& pb) {
    RearrangementRRT::addToTree(new_motion, parent, pb);
    SlicePtr closest_slice = getSlice(new_motion, pb);
    float slice_distance = distanceToSlice(new_motion, closest_slice);
    if (slice_distance > pb.pq.min_slice_distance) { // we discovered a new slice!
        auto new_slice = getNewSlice(new_motion);
        _slices_nn->add(new_slice);
        _slices_list.push_back(new_slice);
        if (_debug_drawer) {
            _debug_drawer->addNewSlice(new_slice);
        }
    } else { // the new motion/state is in the same slice
        closest_slice->addSample(new_motion);
    }
}

SliceBasedOracleRRT::SlicePtr SliceBasedOracleRRT::getSlice(MotionPtr motion, PlanningBlackboard& pb) const {
    if (_slices_nn->size() == 0) {
        return nullptr;
    }
    auto query_slice = getNewSlice(motion);
    auto nearest = _slices_nn->nearest(query_slice);
    ++pb.stats.num_nearest_neighbor_queries;
    cacheSlice(query_slice);
    return nearest;
}

float SliceBasedOracleRRT::distanceToSlice(ompl::planning::essentials::MotionPtr motion, SlicePtr slice) const {
    if (!slice) {
        return std::numeric_limits<float>::max();
    }
    auto query_slice = getNewSlice(motion);
    float distance = (float)_slice_distance_fn.distance(query_slice, slice);
    cacheSlice(query_slice);
    return distance;
}


SliceBasedOracleRRT::SlicePtr SliceBasedOracleRRT::getNewSlice(ompl::planning::essentials::MotionPtr motion) const {
    if (_slices_cache.empty()) {
        return std::make_shared<Slice>(motion, std::bind(&WithinSliceDistance::distance,
                                                         std::ref(_within_slice_distance_fn),
                                                         std::placeholders::_1,
                                                         std::placeholders::_2));
    }
    auto return_value = _slices_cache.top();
    _slices_cache.pop();
    return_value->reset(motion);
    return return_value;
}

void SliceBasedOracleRRT::cacheSlice(SliceBasedOracleRRT::SlicePtr slice) const {
    slice->clear();
    _slices_cache.push(slice);
}

void SliceBasedOracleRRT::projectSliceOnBall(SlicePtr sample_slice,
                                             SliceConstPtr center_slice,
                                             float radius,
                                             PlanningBlackboard& pb)
{
    auto sim_env_state_center = center_slice->repr->getState()->as<mps::planner::ompl::state::SimEnvWorldStateSpace::StateType>();
    auto sim_env_state_sample = sample_slice->repr->getState()->as<mps::planner::ompl::state::SimEnvWorldStateSpace::StateType>();
    // we are projecting slices, so we do not care about the robot state.
    // just set the robot state of the sample to the state of the representative in the center slice
    _state_space->getSubspace(pb.robot_id)->copyState(sim_env_state_sample->getObjectState(pb.robot_id),
                                                      sim_env_state_center->getObjectState(pb.robot_id));
    Eigen::VectorXf dir;
    _state_space->computeDirection(sim_env_state_center, sim_env_state_sample, dir);
    float prev_distance = (float)_slice_distance_fn.distance(sample_slice, center_slice);
    _state_space->copyState(sim_env_state_sample, sim_env_state_center);
    _state_space->shiftState(sim_env_state_sample, radius / prev_distance * dir);
}

SliceBasedOracleRRT::ExtensionCandidateTuple SliceBasedOracleRRT::selectCandidateSlice(
        const std::vector<SliceBasedOracleRRT::ExtensionCandidateTuple> &candidates) const {
    float normalizer = 0.0f;
    for (const auto& candidate : candidates) {
        normalizer += std::get<1>(candidate);
    }
    float random_f = (float)_rng->uniform01();
    float acc_f = 0.0f;
    for (const auto& candidate : candidates) {
        // TODO do we want some other probability here, i.e. not only feasibility?
        acc_f += 1.0f / normalizer * std::get<1>(candidate);
        if (random_f <= acc_f) {
            return candidate;
        }
    }
    // This could happen due to numerical issues
    return candidates.at(candidates.size() - 1);
}

//////////////////////////////////////// GNATSamplingSliceBasedOracleRRT ////////////////////////////////////////////////
// GNATSamplingSliceBasedOracleRRT::GNATSamplingSliceBasedOracleRRT(::ompl::control::SpaceInformationPtr si,
//                                                          mps::planner::pushing::oracle::PushingOraclePtr pushing_oracle,
//                                                          mps::planner::pushing::oracle::RobotOraclePtr robot_oracle,
//                                                          const std::string &robot_name,
//                                                          const oracle::OracleControlSampler::Parameters &params) :
//     SliceBasedOracleRRT(si, pushing_oracle, robot_oracle, robot_name, params)
// {
// }

// GNATSamplingSliceBasedOracleRRT::~GNATSamplingSliceBasedOracleRRT() = default;

// bool GNATSamplingSliceBasedOracleRRT::sample(mps::planner::ompl::planning::essentials::MotionPtr motion,
//                               unsigned int& target_obj_id,
//                               PlanningBlackboard& pb)
// {
//     static const std::string log_prefix("mps::planner::pushing::algorithm::GNATSamplingSliceBasedOracleRRT::sample]");
//     bool is_goal = false;
//     // sample random state with goal biasing
//     if( _rng->uniform01() < pb.pq.goal_bias && pb.pq.goal_region->canSample()){
//         logging::logDebug("Sampling a goal state", log_prefix);
//         pb.pq.goal_region->sampleGoal(motion->getState());
//         target_obj_id = pb.pq.goal_region->sampleTargetObjectIndex();
//         is_goal = true;
//     }else{
//         logging::logDebug("Sampling from GNAT sampler", log_prefix);
//         target_obj_id = 0;
//         auto gnat_tree = std::static_pointer_cast<::ompl::NearestNeighborsGNAT<SlicePtr> >(_slices_nn);
//         ::ompl::RNG rng(_rng->getLocalSeed());
//         auto sampled_slice = gnat_tree->sample(rng);
//         // TODO this variance is completely arbitrary
//         _state_sampler->sampleGaussian(motion->getState(), sampled_slice->repr->getState(), 0.9);
//         target_obj_id = sampleActiveObject(pb);
//     }
//     pb.stats.num_samples++;
//     return is_goal;
// }

////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////// Shortcutter ///////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
Shortcutter::Shortcutter(::ompl::control::SpaceInformationPtr si):
     _si(si)
{
    _state_propagator = std::dynamic_pointer_cast<mps_control::SimEnvStatePropagator>(si->getStatePropagator());
}

Shortcutter::~Shortcutter() = default;

MotionPtr Shortcutter::getNewMotion() {
    if (not _motions_cache.empty()) {
        MotionPtr ptr = _motions_cache.top();
        _motions_cache.pop();
        return ptr;
    }
    return std::make_shared<Motion>(_si);
}

void Shortcutter::cacheMotion(MotionPtr ptr) {
    _motions_cache.push(ptr);
}

void Shortcutter::cacheMotions(std::vector<MotionPtr>& motions) {
    for (auto& motion : motions) {
        _motions_cache.push(motion);
    }
    motions.clear();
}

PathPtr Shortcutter::getNewPath() {
    if (not _path_cache.empty()) {
        PathPtr ptr = _path_cache.top();
        _path_cache.pop();
        return ptr;
    }
    return std::make_shared<Path>(_si);
}

void Shortcutter::cachePath(PathPtr ptr, int clear_id) {
    if (clear_id >= 0) {
        for (unsigned int id = clear_id; id < ptr->getNumMotions(); ++id) {
            cacheMotion(ptr->getMotion(id));
        }
    }
    ptr->clear(); // make sure our path doesn't keep references to any motions 
    _path_cache.push(ptr);
}

bool Shortcutter::forwardPropagatePath(mps::planner::ompl::planning::essentials::PathPtr path,
                                       std::vector<mps::planner::ompl::planning::essentials::MotionPtr>& new_motions,
                                       mps::planner::ompl::planning::essentials::PathPtr old_path,
                                       unsigned int old_path_continuation,
                                       ShortcutQuery& sq) 
{
    bool goal_satisfied = false;
    bool extension_success = true;
    assert(path->getNumMotions() > 0);
    assert(not new_motions.empty());
    // start with prev_motion as last motion of path
    auto prev_motion = path->getMotion(path->getNumMotions() - 1); 
    // first propagate new_motions
    for (auto& new_motion : new_motions) {
        extension_success = _state_propagator->propagate(prev_motion->getState(),
                                                         new_motion->getControl(),
                                                         new_motion->getState());
        if (not extension_success) {
            return false;
        }
        prev_motion = new_motion;
        path->append(new_motion);
        goal_satisfied = sq.goal_region->isSatisfied(new_motion->getState());
        if (goal_satisfied) return true;
    }
    // next finish propagating remaining actions from old path
    unsigned int current_idx = old_path_continuation;
    // as long as we have propagation success and there are actions in the old path left
    while (extension_success && current_idx < old_path->getNumMotions()) {
        auto new_motion = getNewMotion();
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
            if (goal_satisfied) return true; // if yes we are done
        } else { // gonna abort
            cacheMotion(new_motion);
        }
    }
    return false; // if we reached this point, it means we are not reaching a goal anymore
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////// NaiveShortcutter //////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
NaiveShortcutter::NaiveShortcutter(::ompl::control::SpaceInformationPtr si,
                                   mps::planner::pushing::oracle::RobotOraclePtr robot_oracle) :
    Shortcutter(si), _robot_oracle(robot_oracle)
{
}

NaiveShortcutter::~NaiveShortcutter() = default;

void NaiveShortcutter::shortcut(mps::planner::ompl::planning::essentials::PathPtr path,
                                ShortcutQuery& sq,
                                float max_time) {
    std::unordered_map<std::pair<unsigned int, unsigned int>, bool,
                        boost::hash< std::pair< unsigned int, unsigned int> > > previous_pairs;
    bool finished = path->getNumMotions() <= 2;
    auto rnd_gen = mps::planner::util::random::getDefaultRandomGenerator();
    double current_path_cost = path->cost(sq.shortcut_objective).value();
    auto world_state_space = std::dynamic_pointer_cast<mps_state::SimEnvWorldStateSpace>(_si->getStateSpace());
    unsigned int robot_id = world_state_space->getObjectIndex(sq.robot_name);
    _timer.startTimer(max_time);
    while (!_timer.timeOutExceeded() && !finished) {
        // sample a random pair
        unsigned int start_id = rnd_gen->uniformInt(0, path->getNumMotions()); 
        unsigned int end_id = rnd_gen->uniformInt(start_id + 1, path->getNumMotions());
        auto id_pair = std::make_pair(start_id, end_id);
        auto pair_iter = previous_pairs.find(id_pair);
        if (pair_iter != previous_pairs.end()) { 
            continue; // we already tried this pair
        } else {
            previous_pairs[id_pair] = true;
        }
        // shortcut
        // a motion is (action, state) where the action leads to the state
        auto first_wp = path->getMotion(start_id); // contains state from which to start shortcut
        auto snd_wp = path->getMotion(end_id); // contains state we would like to move to
        std::vector<MotionPtr> new_motions; // will contain new actions
        auto new_path = getNewPath(); // will contain new path
        // concat all waypoints up to start_id (incl)
        new_path->concat(path, start_id + 1);
        // do the actual shortcutting by computing robot actions that steer the robot
        computeRobotActions(new_motions, first_wp->getState(), snd_wp->getState(), robot_id);
        // test whether we still achieve the goal
        bool successful_path = forwardPropagatePath(new_path, new_motions, path, end_id, sq);
        auto new_cost = new_path->cost(sq.shortcut_objective).value();
        if (successful_path and (new_cost < current_path_cost)) {  // replace path if new one is better
            cachePath(path, start_id + 1);
            path = new_path;
        } else {
            cachePath(new_path);
            cacheMotions(new_motions);
        }
    }
}

std::string NaiveShortcutter::getName() const {
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
        auto new_motion = getNewMotion();
        auto control = dynamic_cast<mps_control::RealValueParameterizedControl*>(new_motion->getControl());
        control->setParameters(control_param);
        motions.push_back(new_motion);
    }
}


////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////// OracleShortcutter /////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
OracleShortcutter::OracleShortcutter(::ompl::control::SpaceInformationPtr si,
                                     mps::planner::pushing::oracle::RobotOraclePtr robot_oracle,
                                     mps::planner::pushing::oracle::PushingOraclePtr pushing_oracle) :
    Shortcutter(si), _robot_oracle(robot_oracle) ,  _pushing_oracle(pushing_oracle)
{
}

OracleShortcutter::~OracleShortcutter() = default;

void OracleShortcutter::shortcut(mps::planner::ompl::planning::essentials::PathPtr,
                                ShortcutQuery& sq,
                                float max_time) 
{
}

std::string OracleShortcutter::getName() const {
    return "OracleShortcutter";
}
////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////// ShortCutComparer //////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
ShortcutComparer::ShortcutComparer(::ompl::control::SpaceInformationPtr si,
                                    std::vector<ShortcutterPtr>& short_cutters) :
    Shortcutter(si),
    _short_cutters(short_cutters)
{
}

ShortcutComparer::~ShortcutComparer() = default;

void ShortcutComparer::shortcut(mps::planner::ompl::planning::essentials::PathPtr,
                                ShortcutQuery& sq,
                                float max_time) 
{
}

std::string ShortcutComparer::getName() const {
    return "ShortcutComparer";
}
////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////// DebugDrawer ///////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
DebugDrawer::DebugDrawer(sim_env::WorldViewerPtr world_viewer,
                         unsigned int robot_id,
                         const std::vector<unsigned int>& target_ids) :
        DebugDrawer(world_viewer, nullptr, robot_id, target_ids)
{
}

DebugDrawer::DebugDrawer(sim_env::WorldViewerPtr world_viewer,
                         SliceDrawerInterfacePtr slice_drawer,
                         unsigned int robot_id,
                         const std::vector<unsigned int>& target_ids)
{
    _world_viewer = world_viewer;
    _slice_drawer = slice_drawer;
    if (_slice_drawer) {
        _slice_drawer->setDebugDrawer(shared_from_this());
    }
    _robot_id = robot_id;
    _target_ids = target_ids;
}

DebugDrawer::~DebugDrawer() {
    clear();
}

void DebugDrawer::addNewMotion(MotionPtr motion) {
    if (!motion->getParent()) return;
    auto* parent_state = dynamic_cast<ompl::state::SimEnvWorldState*>(motion->getParent()->getState());
    auto* new_state = dynamic_cast<ompl::state::SimEnvWorldState*>(motion->getState());
    auto* parent_object_state = parent_state->getObjectState(_robot_id);
    auto* new_object_state = new_state->getObjectState(_robot_id);
    drawStateTransition(parent_object_state, new_object_state, Eigen::Vector4f(0.4, 0, 0.9, 1));
    parent_object_state = parent_state->getObjectState(_target_ids.at(0)); // TODO also print other objects
    new_object_state = new_state->getObjectState(_target_ids.at(0)); // TODO also draw other target objects
    drawStateTransition(parent_object_state, new_object_state, Eigen::Vector4f(0, 0.7, 0, 1));
    for (unsigned int i = 0; i < new_state->getNumObjects(); ++i) {
        if ((i != _target_ids.at(0)) and (i != _robot_id)) { // TODO also other target objects
            parent_object_state = parent_state->getObjectState(i);
            new_object_state = new_state->getObjectState(i);
            drawStateTransition(parent_object_state, new_object_state, Eigen::Vector4f(0.9, 0.9, 0.9, 0.4));
        }
    }
}

void DebugDrawer::clear(bool clear_slice_drawer) {
    for (auto& handle : _handles) {
        _world_viewer->removeDrawing(handle);
    }
    _handles.clear();
    if (_slice_drawer and clear_slice_drawer) {
        _slice_drawer->clear();
    }
}

void DebugDrawer::drawStateTransition(const ompl::state::SimEnvObjectState *parent_state,
                                      const ompl::state::SimEnvObjectState *new_state,
                                      const Eigen::Vector4f& color) {
    // TODO this is overfit to box2d push planning
    Eigen::VectorXf config = parent_state->getConfiguration();
    Eigen::Vector3f pos_parent(config[0], config[1], 0.0f);
//    Eigen::Vector3f extent(0.1, 0.1, 0.0);
    config = new_state->getConfiguration();
    Eigen::Vector3f pos_child(config[0], config[1], 0.0f);
//    _handles.push_back(_world_viewer->drawBox(pos_parent, extent, true));
    _handles.push_back(_world_viewer->drawLine(pos_parent, pos_child, color, 0.01f));
//    _handles.push_back(_world_viewer->drawBox(pos_child, extent, true));
}

void DebugDrawer::showState(const ompl::state::SimEnvWorldState* state,
                            const ompl::state::SimEnvWorldStateSpaceConstPtr state_space) {
    auto world = _world_viewer->getWorld();
    state_space->setToState(world, state);
}

void DebugDrawer::addNewSlice(mps::planner::pushing::algorithm::SliceBasedOracleRRT::SliceConstPtr slice) {
    if(!_slice_drawer) {
        return;
    }
    _slice_drawer->addSlice(slice);
}

void DebugDrawer::setSliceDrawer(SliceDrawerInterfacePtr slice_drawer) {
    _slice_drawer = slice_drawer;
    if (_slice_drawer) {
        _slice_drawer->setDebugDrawer(shared_from_this());
    }
}

void DebugDrawer::setRobotId(unsigned int robot_id) {
    _robot_id = robot_id;
}

void DebugDrawer::setTargetIds(const std::vector<unsigned int>& target_ids) {
    _target_ids = target_ids;
}

SliceDrawerInterfacePtr DebugDrawer::getSliceDrawer() {
    return _slice_drawer;
}

SliceDrawerInterface::~SliceDrawerInterface() = default;

void SliceDrawerInterface::setDebugDrawer(mps::planner::pushing::algorithm::DebugDrawerPtr debug_drawer) {
    _debug_drawer = debug_drawer;
}

void SliceDrawerInterface::setStateSpace(mps::planner::ompl::state::SimEnvWorldStateSpacePtr state_space) {
    _state_space = state_space;
}

