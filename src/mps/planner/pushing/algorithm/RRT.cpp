//
// Created by joshua on 8/14/17.
//

#include <mps/planner/pushing/algorithm/RRT.h>
#include <mps/planner/util/Logging.h>
#include <mps/planner/ompl/control/Interfaces.h>
#include <ompl/datastructures/NearestNeighborsSqrtApprox.h>

#include <queue>

namespace logging = mps::planner::util::logging;
namespace ob = ::ompl::base;
namespace oc = ::ompl::control;
namespace mps_state = mps::planner::ompl::state;
namespace mps_control = mps::planner::ompl::control;
using namespace mps::planner::pushing::algorithm;
using namespace mps::planner::ompl::planning::essentials;

SemiDynamicRRT::PlanningQuery::PlanningQuery(std::shared_ptr<ob::GoalSampleableRegion> goal_region,
                                             ob::State *start_state,
                                             float time_out,
                                             const std::string& target_name,
                                             const std::string& robot_name) :
    goal_region(goal_region),
    start_state(start_state),
    time_out(time_out),
    target_name(target_name),
    robot_name(robot_name)
{
    stopping_condition = []() {return false;};
    goal_bias = 0.2f;
    robot_bias = 0.0f;
    target_bias = 0.25f;
}


////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////// DebugDrawer ///////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
SemiDynamicRRT::DebugDrawer::DebugDrawer(sim_env::WorldViewerPtr world_viewer) {
    _world_viewer = world_viewer;
}

SemiDynamicRRT::DebugDrawer::~DebugDrawer() {
    clear();
}

void SemiDynamicRRT::DebugDrawer::addNewMotion(MotionPtr motion) {
    auto* parent_state = dynamic_cast<ompl::state::SimEnvWorldState*>(motion->getParent()->getState());
    auto* new_state = dynamic_cast<ompl::state::SimEnvWorldState*>(motion->getState());
    // TODO is it guaranteed that object 0 is the robot?
    auto* parent_object_state = parent_state->getObjectState(0);
    auto* new_object_state = new_state->getObjectState(0);
    drawStateTransition(parent_object_state, new_object_state, Eigen::Vector4f(0, 0, 1, 1));
    // TODO the target object may not be object 1
    parent_object_state = parent_state->getObjectState(1);
    new_object_state = new_state->getObjectState(1);
    drawStateTransition(parent_object_state, new_object_state, Eigen::Vector4f(0, 1, 0, 1));
}

void SemiDynamicRRT::DebugDrawer::clear() {
    for (auto& handle : _handles) {
        _world_viewer->removeDrawing(handle);
    }
    _handles.clear();
}

void SemiDynamicRRT::DebugDrawer::drawStateTransition(const ompl::state::SimEnvObjectState *parent_state,
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
/////////////////////////////////////// SemiDynamicRRT /////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
SemiDynamicRRT::SemiDynamicRRT(::ompl::control::SpaceInformationPtr si) :
        _si(si),
        _log_prefix("[mps::planner::pushing::algorithm::SemiDynamicRRT::"),
        _is_setup(false)
{
}

SemiDynamicRRT::~SemiDynamicRRT() = default;

void SemiDynamicRRT::setup() {
    _state_space = std::dynamic_pointer_cast<mps_state::SimEnvWorldStateSpace>(_si->getStateSpace());
    if (!_state_space) {
        throw std::logic_error(_log_prefix + "setup] Could not cast state space to SimEnvWorldStateSpace");
    }
    _distance_measure = std::make_shared<mps::planner::pushing::PushPlannerDistanceMeasure>(_state_space);
    _state_space->setDistanceMeasure(_distance_measure);
    // TODO set everything up
    if (!_tree) {
        // TODO this is not an efficient data structure, but we need to have a structure that allows modifying the distance function
        // TODO or a data structure for each setting of active objects (in rearrangement planning context)
        _tree = std::make_shared<::ompl::NearestNeighborsSqrtApprox< MotionPtr > >();
        using namespace std::placeholders;
        _tree->setDistanceFunction(std::bind(&SemiDynamicRRT::treeDistanceFunction, this, _1, _2));
    }
    _is_setup = true;
    _control_sampler = _si->allocDirectedControlSampler();
    _state_sampler = _si->allocStateSampler();
    auto state_space = std::dynamic_pointer_cast<mps_state::SimEnvWorldStateSpace>(_si->getStateSpace());
    if (!state_space) {
        throw std::logic_error(_log_prefix + "SemiDynamicRRT] Could not cast state space to SimEnvWorldStateSpace");
    }
    _num_objects = state_space->getNumObjects();
    assert(_num_objects > 1);
    _rng = mps::planner::util::random::getDefaultRandomGenerator();
}

bool SemiDynamicRRT::plan(const PlanningQuery& pq, PathPtr path) {
    static const std::string log_prefix(_log_prefix + "plan]");
    if (not _is_setup) {
        logging::logErr("Planner is not setup, aborting", log_prefix);
        return false;
    }
    logging::logDebug("Starting to plan", log_prefix);
    _distance_measure->setWeights(pq.weights);
    unsigned int robot_id(0);
    unsigned int target_id(1);
    {
        int tmp_robot_id = _state_space->getObjectIndex(pq.robot_name);
        int tmp_target_id = _state_space->getObjectIndex(pq.target_name);
        if (tmp_robot_id < 0 || tmp_target_id < 0) {
            logging::logErr("Could not retrieve ids for robot or target object. Are they in the state space?", log_prefix);
            return false;
        }
        target_id = (unsigned int)tmp_target_id;
        robot_id = (unsigned int)tmp_robot_id;
    }

    if (_debug_drawer) {
        _debug_drawer->clear();
    }

    MotionPtr current_motion = getNewMotion();
    MotionPtr sample = getNewMotion();
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
    // TODO remove debug stream again
    std::stringstream debug_stream;
    // Do the actual planning
    while(not _timer.timeOutExceeded() and not pq.stopping_condition() && !solved) {
        // sample random state with goal biasing
        unsigned int object_id = 0;
        if( _rng->uniform01() < pq.goal_bias && pq.goal_region->canSample()){
            logging::logDebug("Sampling a goal state", log_prefix);
            pq.goal_region->sampleGoal(sample->getState());
            object_id = target_id;
        }else{
            logging::logDebug("Sampling a state uniformly", log_prefix);
            _state_sampler->sampleUniform(sample->getState());
            // Sample which object should be active
            object_id = sampleActiveObject(pq, target_id, robot_id);
        }
        // set only one object active in each iteration
        _distance_measure->setAll(false);
        _distance_measure->setActive(object_id, true);
        // TODO remove this debug print
        auto* world_state = dynamic_cast<mps_state::SimEnvWorldState*>(sample->getState());
        debug_stream.str("");
        debug_stream << "Sampled state: ";
        world_state->print(debug_stream);
        logging::logDebug(debug_stream.str(), log_prefix);
        // TODO until here
        // Get nearest tree node
        logging::logDebug("Searching for nearest neighbor.", log_prefix);
        current_motion = _tree->nearest(sample);
        // TODO remove this debug print
        world_state = dynamic_cast<mps_state::SimEnvWorldState*>(current_motion->getState());
        debug_stream.str("");
        debug_stream << "Nearest state: ";
        world_state->print(debug_stream);
        logging::logDebug(debug_stream.str(), log_prefix);
        // TODO until here
        // Compute action to take us towards the sampled state
        MotionPtr new_motion = getNewMotion();
        _si->copyState(new_motion->getState(), sample->getState());
        // sampleTo(c, is, ns) - samples a control c that moves is towards ns.
        // ns is the overwritten to be the actual outcome (i.e. ns = f(is, c))
        // the return value is either 0 if sampling a valid control failed
        logging::logDebug("Sampling a control", log_prefix);
        unsigned int num_steps = _control_sampler->sampleTo(new_motion->getControl(),
                                                            current_motion->getState(),
                                                            new_motion->getState());
        if (num_steps > 0) { // the sampled control is valid, i.e. the outcoming state is valid
            // TODO remove this debug print
            world_state = dynamic_cast<mps_state::SimEnvWorldState*>(new_motion->getState());
            debug_stream.str("");
            debug_stream << "New state";
            world_state->print(debug_stream);
            logging::logDebug(debug_stream.str(), log_prefix);
            // TODO until here
            logging::logDebug("Sampling control succeeded, adding new state", log_prefix);
            new_motion->setParent(current_motion);
            _tree->add(new_motion);
            solved = pq.goal_region->isSatisfied(new_motion->getState());
            final_motion = new_motion;
            // TODO either remove again or add macro to only add this when build with debug flags
            if (_debug_drawer) {
                _debug_drawer->addNewMotion(new_motion);
            }
        } else {
            logging::logDebug("Could not find a valid control", log_prefix);
            cacheMotion(new_motion);
        }
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

unsigned int SemiDynamicRRT::sampleActiveObject(const PlanningQuery& pq,
                                                unsigned int target_id,
                                                unsigned int robot_id) const {
    double random_value = _rng->uniform01();
    if (random_value < pq.target_bias) {
        return target_id;
    } else if (random_value < pq.target_bias + pq.robot_bias) {
        return robot_id;
    } else {
        return static_cast<unsigned int>(_rng->uniformInt(0, _num_objects - 1));
    }
}

MotionPtr SemiDynamicRRT::getNewMotion() {
    if (not _motions_cache.empty()) {
        MotionPtr ptr = _motions_cache.top();
        _motions_cache.pop();
        return ptr;
    }
    return std::make_shared<Motion>(_si);
}

void SemiDynamicRRT::cacheMotion(MotionPtr ptr) {
    _motions_cache.push(ptr);
}

double SemiDynamicRRT::treeDistanceFunction(const MotionPtr &a, const MotionPtr &b) const {
    auto* state_a = a->getState();
    auto* state_b = b->getState();
    return _state_space->distance(state_a, state_b); // this uses _distance_measure
}

void SemiDynamicRRT::setDebugDrawer(DebugDrawerPtr debug_drawer) {
    _debug_drawer = debug_drawer;
}
