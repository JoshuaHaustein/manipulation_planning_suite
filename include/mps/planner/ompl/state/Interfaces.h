//
// Created by joshua on 8/30/17.
//

#ifndef MANIPULATION_PLANNING_SUITE_INTERFACES_H
#define MANIPULATION_PLANNING_SUITE_INTERFACES_H

#include <memory>
#include <ompl/base/State.h>

namespace mps {
namespace planner {
    namespace ompl {
        namespace state {
            class StateDistanceMeasure {
            public:
                virtual ~StateDistanceMeasure() = 0;
                virtual double distance(const ::ompl::base::State* state1, const ::ompl::base::State* state2) const = 0;
            };

            typedef std::shared_ptr<StateDistanceMeasure> StateDistanceMeasurePtr;
            typedef std::shared_ptr<const StateDistanceMeasure> StateDistanceMeasureConstPtr;
            typedef std::weak_ptr<StateDistanceMeasure> StateDistanceMeasureWeakPtr;
            typedef std::weak_ptr<const StateDistanceMeasure> StateDistanceMeasureWeakConstPtr;
        }
    }
}
}

#endif //MANIPULATION_PLANNING_SUITE_INTERFACES_H
