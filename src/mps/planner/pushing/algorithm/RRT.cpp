//
// Created by joshua on 8/14/17.
//

#include <mps/planner/pushing/algorithm/RRT.h>
#include <mps/planner/util/Logging.h>
#include <mps/planner/ompl/control/Interfaces.h>
//#include <ompl/datastructures/NearestNeighborsGNAT.h>
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
        _log_prefix("[mps::planner::pushing::algorithm::RearrangementRRT::"),
        _is_setup(false)
{
}

RearrangementRRT::~RearrangementRRT() = default;

void RearrangementRRT::setup() {
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
    _is_setup = true;
    _state_sampler = _si->allocStateSampler();
    assert(_state_space->getNumObjects() > 1);
    _rng = mps::planner::util::random::getDefaultRandomGenerator();
}

bool RearrangementRRT::plan(const PlanningQuery& pq, PathPtr path) {
    static const std::string log_prefix(_log_prefix + "plan]");
    if (not _is_setup) {
        logging::logErr("Planner is not setup, aborting", log_prefix);
        return false;
    }
    logging::logDebug("Starting to plan", log_prefix);
    _distance_measure->setWeights(pq.weights);
    PlanningBlackboard blackboard(pq);
    setupBlackboard(blackboard);
    if (_debug_drawer) {
        _debug_drawer->clear();
    }

    MotionPtr current_motion = getNewMotion();
    MotionPtr sample_motion = getNewMotion();
    MotionPtr final_motion = nullptr;
    // set the state to be the start state
    _si->copyState(current_motion->getState(), pq.start_state);
    // set the control to be null
    _si->nullControl(current_motion->getControl());
    // Initialize the tree
    _tree->clear();
    _tree->add(current_motion);
    final_motion = current_motion;

    bool solved = pq.goal_region->isSatisfied(current_motion->getState());
    std::stringstream ss;
    pq.goal_region->print(ss);
    logging::logDebug("Planning towards goal " + ss.str(), log_prefix);
    logging::logDebug("Entering main loop", log_prefix);
    _timer.startTimer(pq.time_out);
    // Do the actual planning
    while(not _timer.timeOutExceeded() and not pq.stopping_condition() && !solved) {
        // sample a new state
        unsigned int active_obj_id = 0;
        bool goal_sampled = sample(sample_motion, active_obj_id, blackboard);
        printState("Sampled state is ", sample_motion->getState()); // TODO remove
        // Get a tree node to expand
        selectTreeNode(sample_motion, current_motion, active_obj_id, goal_sampled, blackboard);
        printState("Selected tree state: ", current_motion->getState()); // TODO remove
        // Extend the tree
        solved = extend(current_motion, sample_motion->getState(), active_obj_id, final_motion, blackboard);
        printState("Tree extended to ", final_motion->getState()); // TODO remove
    }

    logging::logDebug(boost::format("Main loop finished, runtime was %f") % _timer.stopTimer(),
                      log_prefix);
    // create the path if we found a solution
    if(solved){
        logging::logInfo("Found a solution", log_prefix);
        path->initBacktrackMotion(final_motion);
    }
    // clean up
    _tree->clear();

    logging::logInfo("Planning finished", log_prefix);
    return solved;
}

bool RearrangementRRT::sample(mps::planner::ompl::planning::essentials::MotionPtr motion,
                              unsigned int& target_obj_id,
                              const PlanningBlackboard& pb)
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
    return is_goal;
}

void RearrangementRRT::selectTreeNode(const ompl::planning::essentials::MotionPtr& sample_motion,
                                      ompl::planning::essentials::MotionPtr& selected_node,
                                      unsigned int& active_obj_id,
                                      bool sample_is_goal,
                                      const PlanningBlackboard& pb)
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
    _distance_measure->setAll(false); // we take the full state into account here
    if (not sample_is_goal) {
        active_obj_id = sampleActiveObject(pb);
    }
    _distance_measure->setActive(active_obj_id, true);
    selected_node = _tree->nearest(sample_motion);
}

void RearrangementRRT::addToTree(mps::planner::ompl::planning::essentials::MotionPtr new_motion,
                                 mps::planner::ompl::planning::essentials::MotionPtr parent)
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
                                   const PlanningBlackboard& pb)
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
        addToTree(new_motion, start);
        last_motion = new_motion;
    } else {
        logging::logDebug("Could not find a valid control", log_prefix);
        cacheMotion(new_motion);
    }
    return reached_a_goal;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// OracleRearrangementRRT /////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
OracleRearangementRRT::OracleRearangementRRT(::ompl::control::SpaceInformationPtr si,
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

OracleRearangementRRT::~OracleRearangementRRT() = default;

void OracleRearangementRRT::setOracleSamplerParameters(
        const mps::planner::pushing::oracle::OracleControlSampler::Parameters &params) {
    _oracle_sampler.setParameters(params);
}

bool OracleRearangementRRT::extend(MotionPtr start,
                                   ::ompl::base::State *dest,
                                   unsigned int active_obj_id,
                                   MotionPtr &last_motion,
                                   const RearrangementRRT::PlanningBlackboard &pb) {
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
        if (not success) { // we failed, no tree extension
            logging::logDebug("A control provided by the oracle failed. ", log_prefix);
            cacheMotion(new_motion);
            return false;
        }
        printState("Oracle control took us to state ", new_motion->getState());
        // we extended the tree a bit, add this new state to the tree
        addToTree(new_motion, prev_motion);
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


