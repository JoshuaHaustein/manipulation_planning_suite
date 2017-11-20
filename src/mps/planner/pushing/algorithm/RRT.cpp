//
// Created by joshua on 8/14/17.
//

#include <mps/planner/pushing/algorithm/RRT.h>
#include <mps/planner/util/Logging.h>
#include <mps/planner/ompl/control/Interfaces.h>
#include <ompl/datastructures/NearestNeighborsGNAT.h>
#include <ompl/datastructures/NearestNeighborsSqrtApprox.h>

#include <queue>

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
    num_slice_neighbors = 8;
    slice_volume = 0.01;
    max_slice_distance = 0.0;
}

RearrangementRRT::PlanningQuery::PlanningQuery(const PlanningQuery &other) = default;

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
//        _tree = std::make_shared<::ompl::NearestNeighborsGNAT <mps::planner::ompl::planning::essentials::MotionPtr> >();
        _tree = std::make_shared<::ompl::NearestNeighborsSqrtApprox <mps::planner::ompl::planning::essentials::MotionPtr> >();
        using namespace std::placeholders;
        _tree->setDistanceFunction(std::bind(&RearrangementRRT::treeDistanceFunction, this, _1, _2));
    }
    _state_sampler = _si->allocStateSampler();
    assert(_state_space->getNumObjects() > 1);
    _rng = mps::planner::util::random::getDefaultRandomGenerator();
}

RearrangementRRT::~RearrangementRRT() = default;

void RearrangementRRT::setup(const PlanningQuery& pq, PlanningBlackboard& blackboard) {
    setupBlackboard(blackboard);
    _distance_measure->setWeights(pq.weights);
    if (_debug_drawer) {
        _debug_drawer->clear();
    }
    _tree->clear();
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
    _timer.startTimer(pq.time_out);
    // Do the actual planning
    while(not _timer.timeOutExceeded() and not pq.stopping_condition() && !solved) {
        blackboard.stats.num_iterations++;
        // sample a new state
        unsigned int active_obj_id = 0;
        bool goal_sampled = sample(sample_motion, active_obj_id, blackboard);
        printState("Sampled state is ", sample_motion->getState()); // TODO remove
        // Get a tree node to expand
        selectTreeNode(sample_motion, current_motion, active_obj_id, goal_sampled, blackboard);
        printState("Selected tree state: ", current_motion->getState()); // TODO remove
        logging::logDebug(boost::format("Active object is %i") % active_obj_id, log_prefix);
        // Extend the tree
        solved = extend(current_motion, sample_motion->getState(), active_obj_id, final_motion, blackboard);
        printState("Tree extended to ", final_motion->getState()); // TODO remove
    }

    blackboard.stats.runtime = _timer.stopTimer();
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
        _state_sampler->sampleUniform(motion->getState());
        target_obj_id = 0;
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

    ///////////////////////////////////////////////////////////////////////////
    /////////////// VARIANT 1: Whole state space in distance //////////////////
    ////////////// CAN USE ANY NEAREST NEIGHBOR STRUCTURE /////////////////////
//    _distance_measure->setAll(true); // we take the full state into account here
//    selected_node = _tree->nearest(sample_motion);
//    if (not sample_is_goal) {
//        // TODO instead of doing this randomly, we could choose it based on the distance
//        // TODO between current_motion and sample
//        // TODO also, if we want to force the algorithm to explore a slice, we would need to this here
//        active_obj_id = sampleActiveObject(pb);
//    }

    ///////////////////////////////////////////////////////////////////////////
    /////////////// VARIANT 2: We pick an active object first /////////////////
    ////////////// NEEDS LINEAR OR SQRT NEAREST NEIGHBOR //////////////////////
    _distance_measure->setAll(false); // we take only the active object into account here
    if (not sample_is_goal) {
        active_obj_id = sampleActiveObject(pb);
    }
    _distance_measure->setActive(active_obj_id, true);
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
        printState("Extending towards state ", new_motion->getState());
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
/////////////////////////////////// OracleRearrangementRRT /////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
OracleRearrangementRRT::OracleRearrangementRRT(::ompl::control::SpaceInformationPtr si,
                                             mps::planner::pushing::oracle::PushingOraclePtr pushing_oracle,
                                             mps::planner::pushing::oracle::RobotOraclePtr robot_oracle,
                                             const std::string& robot_name,
                                             const oracle::OracleControlSampler::Parameters& params) :
        RearrangementRRT(si),
        _oracle_sampler(si, pushing_oracle, robot_oracle, robot_name, params)
{
    _state_propagator = std::dynamic_pointer_cast<mps_control::SimEnvStatePropagator>(si->getStatePropagator());
    assert(_state_propagator);
}

OracleRearrangementRRT::~OracleRearrangementRRT() = default;

void OracleRearrangementRRT::setOracleSamplerParameters(
        const mps::planner::pushing::oracle::OracleControlSampler::Parameters &params) {
    _oracle_sampler.setParameters(params);
}

bool OracleRearrangementRRT::extend(MotionPtr start,
                                   ::ompl::base::State *dest,
                                   unsigned int active_obj_id,
                                   MotionPtr &last_motion,
                                   RearrangementRRT::PlanningBlackboard &pb) {
    static const std::string log_prefix("[mps::planner::pushing::algorithm::OracleRearrangementRRT]");
    std::vector<const ::ompl::control::Control*> controls;
    _oracle_sampler.sampleTo(controls,
                             start->getState(),
                             dest,
                             active_obj_id);
    if (controls.empty()) {
        logging::logErr("OracleControlSampler provided no controls at all", log_prefix);
    }

    MotionPtr prev_motion = start;
    last_motion = start;
    for (auto const* control : controls) {
        MotionPtr new_motion = getNewMotion();
        _si->copyControl(new_motion->getControl(), control);
        bool success = _state_propagator->propagate(prev_motion->getState(),
                                                    new_motion->getControl(),
                                                    new_motion->getState());
        pb.stats.num_state_propagations++;
        if (not success) { // we failed, no tree extension
            logging::logDebug("A control provided by the oracle failed. ", log_prefix);
            cacheMotion(new_motion);
            return false;
        }
        printState("Oracle control took us to state ", new_motion->getState());
        // we extended the tree a bit, add this new state to the tree
        addToTree(new_motion, prev_motion, pb);
        last_motion = new_motion;
        if (pb.pq.goal_region->isSatisfied(new_motion->getState())) {
            // we reached a goal!
            return true;
        }
        // otherwise we just continue extending as long as we have controls
        prev_motion = new_motion;
    }
    return false;
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
                                         const std::string &robot_name,
                                         const oracle::OracleControlSampler::Parameters &params) :
    OracleRearrangementRRT(si, pushing_oracle, robot_oracle, robot_name, params),
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
    _slice_distance_fn.setRobotId(pb.robot_id);
    _within_slice_distance_fn.setRobotId(pb.robot_id);
    _slices_nn->clear();
    _slices_list.clear();
}

bool SliceBasedOracleRRT::sample(mps::planner::ompl::planning::essentials::MotionPtr motion,
                                 unsigned int& target_obj_id,
                                 PlanningBlackboard& pb)
{
    bool sampled_goal = false;
    ////////////////////////////////// Variant 1 //////////////////////////////////
    // TODO if this variant is better, we don't need to overwrite, because it is actually the default sample method
    auto nu = (float)_rng->uniform01();
    if (nu < pb.pq.goal_bias and pb.pq.goal_region->canSample()) {
        pb.pq.goal_region->sampleGoal(motion->getState());
        sampled_goal = true;
    } else {
        _state_sampler->sampleUniform(motion->getState()); // TODO might need valid state sampler
    }
    target_obj_id = sampleActiveObject(pb);
    pb.stats.num_samples++;

    ////////////////////////////////// Variant 2 //////////////////////////////////
//    if (nu < pb.pq.robot_bias) { // we want to move the robot
//        // Variant 2 - sample a slice from _slices and sample a robot configuration and merge them
//        // TODO
//        unsigned int slice_idx = (unsigned int)(_rng->uniformInt(0, std::max(_slices_list.size() - 1, 0)));
//        auto slice = _slices_list.at(slice_idx):
//        // first set the slice
//        _si->copyState(motion->getState(), slice->repr->getState());
//        // now sample a random robot state within that slice
//        auto world_state = dynamic_cast<mps_state::SimEnvWorldState*>(motion->getState());
//        auto robot_state = world_state->getObjectState(pb.robot_id);
//        _robot_state_sampler->sampleUniform(robot_state); // TODO might need valid state sampler
//        target_obj_id = pb.robot_id;
//    } else if (nu < pb.pq.target_bias + pb.pq.robot_bias and pb.pq.goal_region->canSample()) {
//        // we want to move the target object to the goal
//        pb.pq.goal_region->sampleGoal(motion->getState());
//        target_obj_id = pb.target_id;
//        sampled_goal = true;
//    } else { // we want to explore more slices
//        _state_sampler->sampleUniform(motion->getState());
//        // TODO should we bias this on the target object as well?
//        target_obj_id = (unsigned int)(_rng->uniformInt(0, _state_space->getNumObjects() - 1));
//    }
    return sampled_goal;
}

void SliceBasedOracleRRT::selectTreeNode(const ompl::planning::essentials::MotionPtr& sample,
                                         ompl::planning::essentials::MotionPtr& selected_node,
                                         unsigned int& active_obj_id,
                                         bool sample_is_goal,
                                         PlanningBlackboard& pb)
{
    if (active_obj_id == pb.robot_id) {
        // pick the slice that is closest to the sample
        SlicePtr slice = getSlice(sample);
        // the selected node is the nearest node to the sample within the selected slice (in terms of robot distance)
        selected_node = slice->slice_samples_nn->nearest(sample);
    } else {
        // pick k nearest neighbor slices of sample
        std::vector<SlicePtr> k_neighbors;
        // TODO can we pick k in a good way? Would it make sense to query within some radius?
        getKSlices(sample, pb.pq.num_slice_neighbors, k_neighbors);
        float best_feasibility = std::numeric_limits<float>::lowest();
        for (std::size_t slice_idx = 0; slice_idx < k_neighbors.size(); ++slice_idx) {
            SlicePtr slice_i = k_neighbors.at(slice_idx);
            // search within this slice for a better sample
            std::size_t num_states = slice_i->slice_samples_list.size();
            std::size_t random_offset = (std::size_t) (_rng->uniformInt(0, std::max((int) (num_states) - 1, 0)));
            std::size_t num_checks = (std::size_t) (std::floor(std::sqrt((double) (num_states))));
            // we do not want to run through all, so we step through it in sqrt(n) steps with random initial offset
            // this is inspired by the approximate nearest neighbor search in ompl::NearestNeighborsSqrtApprox
            for (unsigned int state_check = 0; state_check < num_checks; ++state_check) {
                std::size_t state_idx = (random_offset + state_check * num_checks) % num_states;
                MotionPtr state = slice_i->slice_samples_list.at(state_idx);
                float feasibility = evaluateFeasibility(state, sample, pb.robot_id, active_obj_id);
                if (feasibility > best_feasibility) {
                    selected_node = state;
                    best_feasibility = feasibility;
                }
            }
        }

    }
}

void SliceBasedOracleRRT::addToTree(MotionPtr new_motion, MotionPtr parent, PlanningBlackboard& pb) {
    RearrangementRRT::addToTree(new_motion, parent, pb);
    SlicePtr closest_slice = getSlice(new_motion);
    float slice_distance = distanceToSlice(new_motion, closest_slice);
    if (slice_distance > pb.pq.slice_volume) { // we discovered a new slice!
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

void SliceBasedOracleRRT::getKSlices(MotionPtr motion,
                                     unsigned int k,
                                     std::vector<SlicePtr>& slices) const {
    auto query_slice = getNewSlice(motion);
    _slices_nn->nearestK(query_slice, k, slices);
    cacheSlice(query_slice);
}

SliceBasedOracleRRT::SlicePtr SliceBasedOracleRRT::getSlice(MotionPtr motion) const {
    if (_slices_nn->size() == 0) {
        return nullptr;
    }
    auto query_slice = getNewSlice(motion);
    auto nearest = _slices_nn->nearest(query_slice);
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

float SliceBasedOracleRRT::evaluateFeasibility(ompl::planning::essentials::MotionPtr from_motion,
                                               ompl::planning::essentials::MotionPtr to_motion,
                                               unsigned int robot_id,
                                               unsigned int target_id) const
{
    // extract from state
    auto from_world_state = dynamic_cast<mps_state::SimEnvWorldState*>(from_motion->getState());
    auto from_robot_state = from_world_state->getObjectState(robot_id);
    auto from_object_state = from_world_state->getObjectState(target_id);
    // extract to state
    auto to_world_state = dynamic_cast<mps_state::SimEnvWorldState*>(to_motion->getState());
    auto to_object_state = to_world_state->getObjectState(target_id);
    // extract Eigen vectors
    Eigen::VectorXf eigen_current_robot = from_robot_state->getConfiguration();
    Eigen::VectorXf eigen_current_object = from_object_state->getConfiguration();
    Eigen::VectorXf eigen_next_object = to_object_state->getConfiguration();
    // prepare oracle and let it predict feasibility
    ////////////////////////////////////////// Pushability projection //////////////////////////////////////
    Eigen::VectorXf projected_object_state;
    // TODO minimal pushability needs to be externally settable
    _pushing_oracle->projectToPushability(eigen_current_object, eigen_next_object, 1.0f, target_id, projected_object_state);
    return _pushing_oracle->predictFeasibility(eigen_current_robot, eigen_current_object, projected_object_state, target_id);
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

//////////////////////////////////////// CompleteSliceBasedOracleRRT ////////////////////////////////////////////////
CompleteSliceBasedOracleRRT::CompleteSliceBasedOracleRRT(::ompl::control::SpaceInformationPtr si,
                                                         mps::planner::pushing::oracle::PushingOraclePtr pushing_oracle,
                                                         mps::planner::pushing::oracle::RobotOraclePtr robot_oracle,
                                                         const std::string &robot_name,
                                                         const oracle::OracleControlSampler::Parameters &params) :
    SliceBasedOracleRRT(si, pushing_oracle, robot_oracle, robot_name, params)
{

}

CompleteSliceBasedOracleRRT::~CompleteSliceBasedOracleRRT() = default;

void CompleteSliceBasedOracleRRT::setup(const PlanningQuery &pq, PlanningBlackboard &blackboard)
{
    SliceBasedOracleRRT::setup(pq, blackboard);
    if (blackboard.pq.max_slice_distance <= 0.0f) {
        assert(_state_space->getNumObjects() > 1);
        blackboard.pq.max_slice_distance = (_state_space->getNumObjects() - 1) * _pushing_oracle->getMaximalPushingDistance();
    }
}

void CompleteSliceBasedOracleRRT::selectTreeNode(const ompl::planning::essentials::MotionPtr &sample,
                                                 ompl::planning::essentials::MotionPtr &selected_node,
                                                 unsigned int &active_obj_id, bool sample_is_goal,
                                                 PlanningBlackboard &pb)
{
    static const std::string log_prefix("[mps::planner::pushing::algorithm::CompleteSliceBasedOracleRRT]");
    logging::logDebug("Selecting tree node to extend for given sample", log_prefix);
    // pick the slice that is closest to the sample
    SlicePtr nearest_slice = getSlice(sample);
    printState("Rerpresentative of nearest slice is ", nearest_slice->repr->getState());

    if (active_obj_id == pb.robot_id) {
        // the selected node is the nearest node to the sample within the selected slice (in terms of robot distance)
        selected_node = nearest_slice->slice_samples_nn->nearest(sample);
    } else {
        // check whether the closest slice is within max_slice_distance
        float slice_distance = distanceToSlice(sample, nearest_slice);
        auto sample_slice = getNewSlice(sample);
        if (slice_distance > pb.pq.max_slice_distance) {
            // else project it
            projectSliceOnBall(sample_slice, nearest_slice, pb.pq.max_slice_distance, pb);
        }
        // next get all neighbor slices within radius max_slice_distance
        std::vector<ExtensionCandidateTuple> candidate_states;
        std::vector<SlicePtr> candidate_slices;
        printState("Sample slice is ", sample_slice->repr->getState());
        _slices_nn->nearestR(sample_slice, 1.00001f * pb.pq.max_slice_distance, candidate_slices);
        assert(not candidate_slices.empty());
        // due to the projection, there is at least one slice we can extend the search from
        for (auto& candidate_slice : candidate_slices) {
            // run over all and select good robot states for pushing
            auto new_motion = getNewMotion();
            _state_space->copyState(new_motion->getState(), candidate_slice->repr->getState());
            // we do this by sampling a feasible state
            _oracle_sampler.sampleFeasibleState(new_motion->getState(),
                                                sample_slice->repr->getState(),
                                                active_obj_id);
            // and selecting the nearest one
            auto nearest_state = candidate_slice->slice_samples_nn->nearest(new_motion);
            float feasibility = _oracle_sampler.getFeasibility(nearest_state->getState(),
                                                               sample_slice->repr->getState(),
                                                               active_obj_id);
            // save what we found
            candidate_states.emplace_back(std::make_tuple(nearest_state, feasibility, new_motion));
        }
        // from all the slices we took a look at, pick one state
        auto selected_state_tuple = selectStateTuple(candidate_states);
        selected_node = std::get<0>(selected_state_tuple); // we selected a state for extension
        // finally, we also want to save the robot state that we sampled, so adjust the sample state
        _state_space->copySubState(sample->getState(), std::get<2>(selected_state_tuple)->getState(), pb.robot_id);
    }
}

bool CompleteSliceBasedOracleRRT::extend(mps::planner::ompl::planning::essentials::MotionPtr start,
                                         ::ompl::base::State *dest, unsigned int active_obj_id,
                                         mps::planner::ompl::planning::essentials::MotionPtr &last_motion,
                                         PlanningBlackboard &pb)
{
    static const std::string log_prefix("[mps::planner::pushing::algorithm::CompleteSliceBasedOracleRRT::extend]");
    std::vector<const ::ompl::control::Control*> controls;
    // first only move the robot
    _oracle_sampler.steerRobot(controls, start->getState(), dest);
    if (controls.empty()) {
        logging::logErr("OracleControlSampler provided no controls at all", log_prefix);
        return false;
    }
    bool extension_success = false;
    bool b_goal = false;
    extendStep(controls, start, last_motion, pb, extension_success, b_goal);
    // next, if the active object is not the robot, try a push
    if (extension_success and active_obj_id != pb.robot_id and not b_goal) {
        controls.clear();
        _oracle_sampler.steerPushSimple(controls, last_motion->getState(), dest, active_obj_id);
        extendStep(controls, last_motion, last_motion, pb, extension_success, b_goal);
    }
    return b_goal;
}

void CompleteSliceBasedOracleRRT::extendStep(const std::vector<const ::ompl::control::Control*>& controls,
                                             const mps::planner::ompl::planning::essentials::MotionPtr &start_motion,
                                             mps::planner::ompl::planning::essentials::MotionPtr &result_motion,
                                             PlanningBlackboard& pb,
                                             bool& extension_success,
                                             bool& goal_reached)
{
    static const std::string log_prefix("[mps::planner::pushing::algorithm::CompleteSliceBasedOracleRRT::extendStep]");
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
        printState("Oracle control took us to state ", new_motion->getState());
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

void CompleteSliceBasedOracleRRT::projectSliceOnBall(SlicePtr sample_slice,
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

CompleteSliceBasedOracleRRT::ExtensionCandidateTuple CompleteSliceBasedOracleRRT::selectStateTuple(
        const std::vector<CompleteSliceBasedOracleRRT::ExtensionCandidateTuple> &candidates) const {
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

