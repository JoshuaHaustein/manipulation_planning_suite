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
RearrangementRRT::PlanningQuery::PlanningQuery(std::shared_ptr<ob::GoalSampleableRegion> goal_region,
                                             ob::State *start_state,
                                             float time_out,
                                             const std::string& target_name,
                                             const std::string& robot_name) :
    goal_region(goal_region),
    start_state(start_state),
    target_name(target_name),
    robot_name(robot_name),
    time_out(time_out)
{
    stopping_condition = []() {return false;};
    goal_bias = 0.2f;
    robot_bias = 0.0f;
    target_bias = 0.25f;
    num_slice_neighbors = 8;
    slice_volume = 0.0;
}

RearrangementRRT::PlanningQuery::PlanningQuery(const PlanningQuery &other) = default;

////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// PlanningBlackboard ////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
RearrangementRRT::PlanningBlackboard::PlanningBlackboard(PlanningQuery pq) :
        pq(pq),
        target_id(1),
        robot_id(0)
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////// DebugDrawer ///////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
RearrangementRRT::DebugDrawer::DebugDrawer(sim_env::WorldViewerPtr world_viewer,
                                           unsigned int robot_id,
                                           unsigned int target_id) {
    _world_viewer = world_viewer;
    _robot_id = robot_id;
    _target_id = target_id;
}

RearrangementRRT::DebugDrawer::~DebugDrawer() {
    clear();
}

void RearrangementRRT::DebugDrawer::addNewMotion(MotionPtr motion) {
    if (!motion->getParent()) return;
    auto* parent_state = dynamic_cast<ompl::state::SimEnvWorldState*>(motion->getParent()->getState());
    auto* new_state = dynamic_cast<ompl::state::SimEnvWorldState*>(motion->getState());
    auto* parent_object_state = parent_state->getObjectState(_robot_id);
    auto* new_object_state = new_state->getObjectState(_robot_id);
    drawStateTransition(parent_object_state, new_object_state, Eigen::Vector4f(0.1, 0, 0.7, 1));
    parent_object_state = parent_state->getObjectState(_target_id);
    new_object_state = new_state->getObjectState(_target_id);
    drawStateTransition(parent_object_state, new_object_state, Eigen::Vector4f(0, 0.7, 0, 1));
}

void RearrangementRRT::DebugDrawer::clear() {
    for (auto& handle : _handles) {
        _world_viewer->removeDrawing(handle);
    }
    _handles.clear();
}

void RearrangementRRT::DebugDrawer::drawStateTransition(const ompl::state::SimEnvObjectState *parent_state,
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
    _distance_measure = std::make_shared<mps::planner::pushing::PushPlannerDistanceMeasure>(_state_space);
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
        target_obj_id = pb.target_id;
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
        return pb.target_id;
    } else if (random_value < pb.pq.target_bias + pb.pq.robot_bias) {
        return pb.robot_id;
    } else {
        return static_cast<unsigned int>(_rng->uniformInt(0, _state_space->getNumObjects() - 1));
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
    pb.target_id = 1;
    {
        int tmp_robot_id = _state_space->getObjectIndex(pb.pq.robot_name);
        int tmp_target_id = _state_space->getObjectIndex(pb.pq.target_name);
        if (tmp_robot_id < 0 || tmp_target_id < 0) {
            throw std::logic_error("[mps::planner::pushing::oracle::RearrangementRRT::setupBlackboard]"
                    "Could not retrieve ids for robot or target object. Are they in the state space?");
        }
        pb.target_id = (unsigned int)tmp_target_id;
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
    slice_samples_nn = std::make_shared<::ompl::NearestNeighborsSqrtApprox <mps::planner::ompl::planning::essentials::MotionPtr> >();
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

double SliceBasedOracleRRT::SliceDistance::distance(const SlicePtr &slice_a, const SlicePtr &slice_b) const
{
    return distance_measure.distance(slice_a->repr->getState(), slice_b->repr->getState());
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
                                              _slice_distance_fn,
                                              std::placeholders::_1,
                                              std::placeholders::_2));
    _query_slice = std::make_shared<Slice>(MotionPtr(), std::function<double(MotionPtr, MotionPtr)>());

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
    _query_slice->repr = nullptr;
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
        target_obj_id = pb.target_id;
        sampled_goal = true;
    } else {
        target_obj_id = sampleActiveObject(pb);
        _state_sampler->sampleUniform(motion->getState()); // TODO might need valid state sampler
    }

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
        auto new_slice = std::make_shared<Slice>(new_motion,
            std::bind(&SliceBasedOracleRRT::WithinSliceDistance::distance, _within_slice_distance_fn,
                      std::placeholders::_1, std::placeholders::_2));
        _slices_nn->add(new_slice);
        _slices_list.push_back(new_slice);
    } else { // the new motion/state is in the same slice
        closest_slice->addSample(new_motion);
    }
}

void SliceBasedOracleRRT::getKSlices(MotionPtr motion,
                                     unsigned int k,
                                     std::vector<SlicePtr>& slices) const {
    _query_slice->repr = motion;
    _slices_nn->nearestK(_query_slice, k, slices);
}

SliceBasedOracleRRT::SlicePtr SliceBasedOracleRRT::getSlice(MotionPtr motion) const {
    if (_slices_nn->size() == 0) {
        return nullptr;
    }
    _query_slice->repr = motion;
    return _slices_nn->nearest(_query_slice);
}

float SliceBasedOracleRRT::distanceToSlice(ompl::planning::essentials::MotionPtr motion, SlicePtr slice) const {
    if (!slice) {
        return std::numeric_limits<float>::max();
    }
    _query_slice->repr = motion;
    return (float)_slice_distance_fn.distance(_query_slice, slice);
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
    _pushing_oracle->prepareOracle(eigen_current_robot, eigen_current_object, eigen_next_object);
    return _pushing_oracle->predictFeasibility(eigen_current_robot, eigen_current_object, eigen_next_object);
}
