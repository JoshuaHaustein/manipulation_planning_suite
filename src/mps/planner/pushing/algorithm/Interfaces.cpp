#include <mps/planner/pushing/algorithm/Interfaces.h>
#include <ompl/datastructures/NearestNeighborsGNAT.h>

namespace logging = mps::planner::util::logging;
namespace ob = ::ompl::base;
namespace oc = ::ompl::control;
namespace mps_state = mps::planner::ompl::state;
namespace mps_control = mps::planner::ompl::control;
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
 ************************************  RobotStateDistanceFn  **********************************
 **********************************************************************************************/
RobotStateDistanceFn::RobotStateDistanceFn(ompl::state::SimEnvWorldStateSpacePtr state_space,
    const std::vector<float>& weights)
    : distance_measure(state_space, weights)
{
}

void RobotStateDistanceFn::setRobotId(unsigned int id)
{
    distance_measure.setAll(false);
    distance_measure.setActive(id, true);
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

RearrangementPlanner::~RearrangementPlanner() = default;

RearrangementPlanner::PlanningQueryPtr RearrangementPlanner::createPlanningQuery(
    mps_state::goal::ObjectsRelocationGoalPtr goal_region,
    mps_state::SimEnvWorldState* start_state,
    const std::string& robot_name, float timeout)
{
    return std::shared_ptr<RearrangementPlanner::PlanningQuery>(new RearrangementPlanner::PlanningQuery(goal_region, start_state, robot_name, timeout));
}