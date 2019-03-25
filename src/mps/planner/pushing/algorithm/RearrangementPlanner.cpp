#include <mps/planner/pushing/algorithm/RearrangementPlanner.h>
#include <ompl/datastructures/NearestNeighborsGNAT.h>

namespace logging = mps::planner::util::logging;
namespace ob = ::ompl::base;
namespace oc = ::ompl::control;
namespace mps_state = mps::planner::ompl::state;
using namespace mps::planner::pushing::algorithm;
using namespace mps::planner::ompl::planning::essentials;

/**********************************************************************************************
 ****************************************  PlanninqQuery  *************************************
 **********************************************************************************************/
RearrangementPlanner::PlanningQuery::PlanningQuery(ompl::state::goal::ObjectsRelocationGoalPtr goal_region,
    mps_state::SimEnvWorldState* start_state,
    const std::string& robot_name,
    float time_out)
    : goal_region(goal_region)
    , start_state(start_state)
    , robot_name(robot_name)
    , time_out(time_out)
{
    stopping_condition = []() { return false; };
    path = nullptr;
    parameters = std::make_shared<ob::ParamSet>();
    // robot_bias = 0.0f;
    // target_bias = 0.25f;
    // do_slice_ball_projection = true;
    // max_slice_distance = 0.0;
}

RearrangementPlanner::PlanningQuery::PlanningQuery(const PlanningQuery& other) = default;

std::string RearrangementPlanner::PlanningQuery::toString() const
{
    std::stringstream ss;
    // TODO for completeness should also print target and start state
    ss << "Robot name: " << robot_name << "\n";
    ss << "timeout: " << time_out << "\n";
    ss << "distance weights: ";
    for (auto& weight : weights) {
        ss << weight << ", ";
    }
    ss << "\n";
    parameters->print(ss);
    return ss.str();
}

/**********************************************************************************************
 ************************************  PlanningBlackboard  ************************************
 **********************************************************************************************/
RearrangementPlanner::PlanningBlackboard::PlanningBlackboard(PlanningQueryPtr pq)
    : pq(pq)
    , robot_id(0)
{
}

/**********************************************************************************************
 ************************************  RearrangementPlanner  **********************************
 **********************************************************************************************/
RearrangementPlanner::RearrangementPlanner(::ompl::control::SpaceInformationPtr si)
    : _si(si)
    , _timer(std::make_shared<mps::planner::util::time::Timer>())
{
    auto state_space = _si->getStateSpace();
    _state_space = std::dynamic_pointer_cast<mps_state::SimEnvWorldStateSpace>(state_space);
    if (_state_space == nullptr) {
        throw std::logic_error("[mps::planner::pushing::algorithm::RearrangementPlanner::setupBlackboard] Invalid state space. State space needs to be SimEnvWorldStateSpace.");
    }
}

RearrangementPlanner::~RearrangementPlanner() = default;

RearrangementPlanner::PlanningQueryPtr RearrangementPlanner::createPlanningQuery(
    mps_state::goal::ObjectsRelocationGoalPtr goal_region,
    mps_state::SimEnvWorldState* start_state,
    const std::string& robot_name, float timeout)
{
    return std::shared_ptr<RearrangementPlanner::PlanningQuery>(new RearrangementPlanner::PlanningQuery(goal_region, start_state, robot_name, timeout));
}

bool RearrangementPlanner::plan(PlanningQueryPtr pq)
{
    PlanningStatistics stats;
    return plan(pq, stats);
}

void RearrangementPlanner::setDebugDrawer(DebugDrawerPtr debug_drawer)
{
    _debug_drawer = debug_drawer;
}

void RearrangementPlanner::setupBlackboard(PlanningBlackboard& pb)
{
    pb.robot_id = 0;
    {
        int tmp_robot_id = _state_space->getObjectIndex(pb.pq->robot_name);
        if (tmp_robot_id < 0) {
            throw std::logic_error("[mps::planner::pushing::oracle::RearrangementPlanner::setupBlackboard]"
                                   "Could not retrieve id for robot "
                + pb.pq->robot_name);
        }
        pb.robot_id = (unsigned int)tmp_robot_id;
    }
}

void RearrangementPlanner::printState(const std::string& msg, ::ompl::base::State* state) const
{
    std::stringstream ss;
    auto* world_state = dynamic_cast<mps_state::SimEnvWorldState*>(state);
    ss.str("");
    ss << msg;
    world_state->print(ss);
    logging::logDebug(ss.str(), "[mps::planner::pushing::oracle::RearrangementPlanner::printState]");
#ifdef DEBUG_VISUALIZE
    if (_debug_drawer)
        _debug_drawer->showState(world_state, _state_space);
#endif
}

/**********************************************************************************************
 ************************************  RobotStateDistanceFn  **********************************
 **********************************************************************************************/
RobotStateDistanceFn::RobotStateDistanceFn(ompl::state::SimEnvWorldStateSpacePtr state_space)
    : distance_measure(state_space, 0)
{
}

void RobotStateDistanceFn::setRobotId(unsigned int id)
{
    distance_measure.setObjectId(id);
}

double RobotStateDistanceFn::distance(const ompl::planning::essentials::MotionPtr& motion_a,
    const ompl::planning::essentials::MotionPtr& motion_b) const
{
    return distance_measure.distance(motion_a->getState(), motion_b->getState());
}

/**********************************************************************************************
*********************************  ObjectArrangementDistanceFn  *******************************
 **********************************************************************************************/
ObjectArrangementDistanceFn::ObjectArrangementDistanceFn(ompl::state::SimEnvWorldStateSpacePtr state_space,
    const std::vector<float>& weights)
    : distance_measure(state_space, weights)
{
}

void ObjectArrangementDistanceFn::setRobotId(unsigned int id)
{
    distance_measure.setAll(true);
    distance_measure.setActive(id, false);
}

double ObjectArrangementDistanceFn::distance(const SliceConstPtr& slice_a, const SliceConstPtr& slice_b) const
{
    double dist = distance_measure.distance(slice_a->repr->getState(), slice_b->repr->getState());
    return dist;
}

/**********************************************************************************************
 *******************************************  Slice  ******************************************
 **********************************************************************************************/
Slice::Slice(mps::planner::ompl::planning::essentials::MotionPtr repr_in,
    StateDistanceFn distance_fn)
{
    slice_samples_nn = std::make_shared<::ompl::NearestNeighborsGNAT<mps::planner::ompl::planning::essentials::MotionPtr>>();
    slice_samples_nn->setDistanceFunction(distance_fn);
    repr = repr_in;
    slice_samples_nn->add(repr);
    slice_samples_list.push_back(repr);
}

Slice::~Slice() = default;

void Slice::addSample(ompl::planning::essentials::MotionPtr motion)
{
    slice_samples_list.push_back(motion);
    slice_samples_nn->add(motion);
}

void Slice::clear()
{
    repr = nullptr;
    slice_samples_nn->clear();
    slice_samples_list.clear();
}
void Slice::reset(ompl::planning::essentials::MotionPtr repr)
{
    clear();
    this->repr = repr;
    slice_samples_nn->add(repr);
    slice_samples_list.push_back(repr);
}

/**************************************************************************************************/
/**************************************** SliceCache **********************************************/
/**************************************************************************************************/
SliceCache::SliceCache(RobotStateDistanceFnPtr dist_fn)
    : _dist_fn(dist_fn)
{
}

SliceCache::~SliceCache() = default;

SlicePtr SliceCache::getNewSlice(ompl::planning::essentials::MotionPtr motion)
{
    if (_slices_cache.empty()) {
        return std::make_shared<Slice>(motion, std::bind(&RobotStateDistanceFn::distance, _dist_fn, std::placeholders::_1, std::placeholders::_2));
    }
    auto return_value = _slices_cache.top();
    _slices_cache.pop();
    return_value->reset(motion);
    return return_value;
}

void SliceCache::cacheSlice(SlicePtr slice)
{
    slice->clear();
    _slices_cache.push(slice);
}
/**************************************************************************************************/
/**************************************** DebugDrawer *********************************************/
/**************************************************************************************************/
DebugDrawer::DebugDrawer(sim_env::WorldViewerPtr world_viewer,
    unsigned int robot_id,
    const std::vector<unsigned int>& target_ids)
    : DebugDrawer(world_viewer, nullptr, robot_id, target_ids)
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

DebugDrawer::~DebugDrawer()
{
    clear();
}

void DebugDrawer::addNewMotion(MotionPtr motion)
{
    if (!motion->getParent())
        return;
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

void DebugDrawer::clear(bool clear_slice_drawer)
{
    for (auto& handle : _handles) {
        _world_viewer->removeDrawing(handle);
    }
    _handles.clear();
    if (_slice_drawer and clear_slice_drawer) {
        _slice_drawer->clear();
    }
}

void DebugDrawer::drawStateTransition(const ompl::state::SimEnvObjectState* parent_state,
    const ompl::state::SimEnvObjectState* new_state,
    const Eigen::Vector4f& color)
{
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
    const ompl::state::SimEnvWorldStateSpaceConstPtr state_space)
{
    auto world = _world_viewer->getWorld();
    state_space->setToState(world, state);
}

void DebugDrawer::addNewSlice(mps::planner::pushing::algorithm::SliceConstPtr slice)
{
    if (!_slice_drawer) {
        return;
    }
    _slice_drawer->addSlice(slice);
}

void DebugDrawer::setSliceDrawer(SliceDrawerInterfacePtr slice_drawer)
{
    _slice_drawer = slice_drawer;
    if (_slice_drawer) {
        _slice_drawer->setDebugDrawer(shared_from_this());
    }
}

void DebugDrawer::setRobotId(unsigned int robot_id)
{
    _robot_id = robot_id;
}

void DebugDrawer::setTargetIds(const std::vector<unsigned int>& target_ids)
{
    _target_ids = target_ids;
}

SliceDrawerInterfacePtr DebugDrawer::getSliceDrawer()
{
    return _slice_drawer;
}

SliceDrawerInterface::~SliceDrawerInterface() = default;

void SliceDrawerInterface::setDebugDrawer(mps::planner::pushing::algorithm::DebugDrawerPtr debug_drawer)
{
    _debug_drawer = debug_drawer;
}

void SliceDrawerInterface::setStateSpace(mps::planner::ompl::state::SimEnvWorldStateSpacePtr state_space)
{
    _state_space = state_space;
}