//
// Created by joshua on 03/20/19.
//

#ifndef MANIPULATION_PLANNING_SUITE_OBJECT_CENTRIC_RRT_H
#define MANIPULATION_PLANNING_SUITE_OBJECT_CENTRIC_RRT_H

// MPS includes
#include <mps/planner/pushing/algorithm/RearrangementPlanner.h>
#include <mps/planner/pushing/oracle/OracleControlSampler.h>

namespace mps {
namespace planner {
    namespace pushing {
        namespace algorithm {
            /**
             * A PushMotion is a Motion that in addition to a state and control, 
             * stores the id of the target object that is supposed to be moved by the control.
             * It also stores a special flag, isTeleportTransit(), that indicates that the robot
             * is transported outside of the pushing plane, i.e. it can not collide with
             * any objects during the control.
             */
            class PushMotion : public mps::planner::ompl::planning::essentials::Motion {
            public:
                PushMotion() = delete;
                PushMotion(::ompl::control::SpaceInformationPtr si);
                PushMotion(const PushMotion& other);
                ~PushMotion();
                PushMotion& operator=(const PushMotion& other);
                void setTargetId(unsigned int id);
                unsigned int getTargetId() const;
                void setTeleportTransit(bool btransit);
                bool isTeleportTransit() const;

            private:
                unsigned int _target_id;
                bool _is_teleport_transit;
            };
            typedef std::shared_ptr<PushMotion> PushMotionPtr;
            typedef std::shared_ptr<const PushMotion> PushMotionConstPtr;
            typedef std::weak_ptr<PushMotion> PushMotionWeakPtr;
            typedef std::weak_ptr<const PushMotion> PushMotionWeakConstPtr;

            /**
             * This class implements an object-centric, semi-dynamic rearrangement RRT algorithm. 
             * The algorithm combines several concepts to efficiently compute rearrangement solutions:
             *  1. To sample pushing states and actions it uses a learned pushing policy and generator.
             *  2. To model the effects of pushing, it uses a physics model.
             *  3. In contrast to a SingleExtendRRT algorithm, this algorithm utilizes a stronger extend function
             *     that attempts to transport all movables to their respective goal configurations. The extend function
             *     greedily pushes movables towards their goals and uses depth-limited backtracking when it fails.
             */
            class MultiExtendRRT : public RearrangementPlanner {
            public:
                MultiExtendRRT(::ompl::control::SpaceInformationPtr si,
                    mps::planner::pushing::oracle::PushingOraclePtr pushing_oracle,
                    mps::planner::pushing::oracle::RobotOraclePtr robot_oracle,
                    const std::string& robot_name);
                ~MultiExtendRRT() override;
                bool plan(PlanningQueryPtr pq, PlanningStatistics& stats) override;
                PlanningQueryPtr createPlanningQuery(mps::planner::ompl::state::goal::ObjectsRelocationGoalPtr goal_region,
                    mps::planner::ompl::state::SimEnvWorldState* start_state, const std::string& robot_name, float timeout) override;
                // void setup(const PlanningQuery& pq, PlanningBlackboard& blackboard) override;
                // bool extend(mps::planner::ompl::planning::essentials::MotionPtr start,
                //     ::ompl::base::State* dest,
                //     unsigned int active_obj_id,
                //     mps::planner::ompl::planning::essentials::MotionPtr& last_motion,
                //     PlanningBlackboard& pb);
                // void selectTreeNode(const ompl::planning::essentials::MotionPtr& sample,
                //     ompl::planning::essentials::MotionPtr& selected_node,
                //     unsigned int& active_obj_id,
                //     bool sample_is_goal,
                //     PlanningBlackboard& pb) override;
                // void addToTree(mps::planner::ompl::planning::essentials::MotionPtr new_motion,
                //     mps::planner::ompl::planning::essentials::MotionPtr parent,
                //     PlanningBlackboard& pb) override;

                void setGoalBias(float goal_bias);
                float getGoalBias() const;

            protected:
                // member functions
                SlicePtr getSlice(ompl::planning::essentials::MotionPtr motion, PlanningBlackboard& pb) const;
                float distanceToSlice(ompl::planning::essentials::MotionPtr motion, SlicePtr slice) const;
                // member variables
                std::vector<SlicePtr> _slices_list;
                std::shared_ptr<::ompl::NearestNeighbors<SlicePtr>> _slices_nn;
                ::ompl::base::StateSamplerPtr _robot_state_sampler;
                ::ompl::base::StateSpacePtr _robot_state_space;
                const RobotStateDistanceFnPtr _robot_state_dist_fn;
                const ObjectArrangementDistanceFnPtr _slice_distance_fn;
                mps::planner::pushing::oracle::PushingOraclePtr _pushing_oracle;
                float _min_slice_distance;
                float _goal_bias;
                // MotionCache<PushMotion> _motion_cache;
                // mutable SliceCache _slice_cache; // Needs to be defined after _robot_state_dist_fn
                // private:
            };
        }
    }
}
}
#endif //MANIPULATION_PLANNING_SUITE_RRT_H
