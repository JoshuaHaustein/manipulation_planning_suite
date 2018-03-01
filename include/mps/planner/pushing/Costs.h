#ifndef MANIPULATION_PLANNING_SUITE_COSTS_H
#define MANIPULATION_PLANNING_SUITE_COSTS_H

#include <mps/planner/ompl/planning/Essentials.h>

namespace mps {
    namespace planner {
        namespace pushing {
            namespace costs {
                class ActionDurationCost : public ompl::planning::essentials::CostFunction {
                    public:
                        ActionDurationCost();
                        ~ActionDurationCost();

                        double cost(ompl::planning::essentials::MotionPtr first,
                                    ompl::planning::essentials::MotionPtr second) override;
                };

                typedef std::shared_ptr<ActionDurationCost> ActionDurationCostPtr;
            }
        }
    }
}

#endif // MANIPULATION_PLANNING_SUITE_COSTS_H