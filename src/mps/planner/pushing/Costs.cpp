#include <mps/planner/ompl/control/Interfaces.h>
#include <mps/planner/pushing/Costs.h>

using namespace mps::planner::pushing::costs;
using namespace mps::planner::ompl::planning::essentials;
using namespace mps::planner::ompl::control;

ActionDurationCost::ActionDurationCost() = default;

ActionDurationCost::~ActionDurationCost() = default;

double ActionDurationCost::cost(ompl::planning::essentials::MotionPtr first,
    ompl::planning::essentials::MotionPtr second)
{
    auto action = dynamic_cast<VelocityControl*>(second->getControl());
    return action->getDuration();
}