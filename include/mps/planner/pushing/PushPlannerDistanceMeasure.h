//
// Created by joshua on 8/30/17.
//

#ifndef MANIPULATION_PLANNING_SUITE_PUSHPLANNERDISTANCEMEASURE_H
#define MANIPULATION_PLANNING_SUITE_PUSHPLANNERDISTANCEMEASURE_H

#include <mps/planner/ompl/state/Interfaces.h>
#include <mps/planner/ompl/state/SimEnvState.h>
#include <vector>

namespace mps {
    namespace planner {
        namespace pushing {
            /**
             * A distance measure used for push planning. This distance measure can be used to guide
             * a motion planner to explore a SimEnvWorldStateSpace. The distance measure can be modified online
             * by enabling/disabling any component of the world state space, i.e. consider object i in the distance or not.
             */
            class PushPlannerDistanceMeasure : public ompl::state::StateDistanceMeasure {
            public:
                PushPlannerDistanceMeasure(ompl::state::SimEnvWorldStateSpacePtr state_space,
                                           const std::vector<float>& weights=std::vector<float>());
                ~PushPlannerDistanceMeasure();

                void setWeights(const std::vector<float>& weights);
                // call to compute distance
                double distance(const ::ompl::base::State* state1, const ::ompl::base::State* state2) const override;

                // Functions to set object i active
                void setActive(unsigned int i, bool active);
                void setActive(const std::string& object_name, bool active);
                void setAll(bool active);
                bool isActive(unsigned int i) const;
                bool isActive(const std::string& object_name) const;
            private:
                ompl::state::SimEnvWorldStateSpaceWeakPtr _weak_state_space;
                std::vector<float> _weights;
                std::vector<bool> _active_flags;
                mps::planner::ompl::state::SimEnvWorldStateSpacePtr getStateSpace() const;
            };
            typedef std::shared_ptr<PushPlannerDistanceMeasure> PushPlannerDistanceMeasurePtr;
            typedef std::shared_ptr<const PushPlannerDistanceMeasure> PushPlannerDistanceMeasureConstPtr;
            typedef std::weak_ptr<PushPlannerDistanceMeasure> PushPlannerDistanceMeasureWeakPtr;
            typedef std::weak_ptr<const PushPlannerDistanceMeasure> PushPlannerDistanceMeasureWeakConstPtr;
        }
    }
}
#endif //MANIPULATION_PLANNING_SUITE_PUSHPLANNERDISTANCEMEASURE_H
