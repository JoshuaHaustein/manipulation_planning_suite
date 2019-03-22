#include <mps/planner/pushing/algorithm/MultiExtendRRT.h>

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

/*************************************** ObjectCentericRRT *****************************************/
MultiExtendRRT::MultiExtendRRT(oc::SpaceInformationPtr si, oracle::PushingOraclePtr pushing_oracle,
    oracle::RobotOraclePtr robot_oracle, const std::string& robot_name)
{
    // TODO
}

MultiExtendRRT::~MultiExtendRRT() = default;

bool MultiExtendRRT::plan(PlanningQueryPtr pq, PlanningStatistics& stats)
{
    return false;
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
