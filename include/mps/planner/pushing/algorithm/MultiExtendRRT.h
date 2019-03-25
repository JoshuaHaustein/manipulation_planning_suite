//
// Created by joshua on 03/20/19.
//

#ifndef MANIPULATION_PLANNING_SUITE_OBJECT_CENTRIC_RRT_H
#define MANIPULATION_PLANNING_SUITE_OBJECT_CENTRIC_RRT_H

// MPS includes
#include <mps/planner/pushing/algorithm/RearrangementPlanner.h>
#include <mps/planner/pushing/oracle/OracleControlSampler.h>
// stl includes
#include <set>
#include <tuple>

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

            class GreedyMonotoneExtension {
            public:
                GreedyMonotoneExtension(mps::planner::pushing::oracle::PushingOraclePtr pushing_oracle,
                    mps::planner::pushing::oracle::RobotOraclePtr robot_oracle);
                ~GreedyMonotoneExtension();
                enum class ExtensionProgress {
                    FAIL = 0,
                    REACHED = 1
                };
                // Setup the extension function
                void setup(unsigned int robot_id, float goal_tolerance, float disturbance_tolerance);
                ExtensionProgress extend(SlicePtr start_slice, mps::planner::ompl::state::SimEnvWorldState const* target,
                    std::vector<PushMotionPtr>& path);

            protected:
                enum class PushResult {
                    FAIL = 0,
                    MOVABLES_BLOCK_PUSH = 1,
                    PROGRESS = 2,
                    REACHED = 3
                };
                typedef std::set<unsigned int> MovableSet;
                // extension step sub-methods
                virtual std::tuple<ExtensionProgress, PushMotionPtr> recursiveExtend(unsigned int t, PushMotionPtr x_c,
                    MovableSet& targets,
                    MovableSet& remainers,
                    mps::planner::ompl::state::SimEnvWorldState const* target_slice);
                virtual std::tuple<PushResult, PushMotionPtr> tryPush(unsigned int t, PushMotionConstPtr x_c,
                    MovableSet& movables, MovableSet& blockers, mps::planner::ompl::state::SimEnvWorldState const* target_slice,
                    bool new_push);
                bool isCloseEnough(PushMotionPtr x_c, mps::planner::ompl::state::SimEnvWorldState const* goal, unsigned int t) const;
                bool madeProgress(PushMotionPtr x_b, PushMotionPtr x_a, mps::planner::ompl::state::SimEnvWorldState const* goal, unsigned int t) const;
                void getPushBlockers(PushMotionPtr x_b, PushMotionPtr x_a, mps::planner::ompl::state::SimEnvWorldState const* goal);

                // member variables
                unsigned int _robot_id;
                unsigned int _num_movables;
                float _goal_tolerance; // max distance an object is allowed to be from its goal to be considered reached
                float _disturbance_tolerance; // max distance an object may be pushed away from its goal
            };

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

                void setGoalBias(float goal_bias);
                float getGoalBias() const;

            protected:
                // member functions
                // high-level algorithm stubs
                virtual bool sample(const PushMotionPtr& sample, PlanningBlackboard& pb);
                virtual void select(const PushMotionPtr& sample, SlicePtr& selected_slice, PlanningBlackboard& pb) const;
                virtual bool extend(const SlicePtr& current, ::ompl::base::State* dest,
                    PushMotionPtr& last_motion, PlanningBlackboard& pb);
                // helpers
                virtual void setup(PlanningQueryPtr pq, PlanningBlackboard& pb);
                void addToTree(PushMotionPtr new_motion, PushMotionPtr parent, PlanningBlackboard& pb);
                SlicePtr getSlice(ompl::planning::essentials::MotionPtr motion, PlanningBlackboard& pb) const;
                float distanceToSlice(ompl::planning::essentials::MotionPtr motion, SlicePtr slice) const;
                // member variables
                const RobotStateDistanceFnPtr _robot_state_dist_fn; // distance fn within slices
                const ObjectArrangementDistanceFnPtr _slice_distance_fn; // distance fn between slices
                mps::planner::pushing::oracle::PushingOraclePtr _pushing_oracle; // pushing oracle
                float _min_slice_distance;
                float _goal_bias;
                mutable MotionCache<PushMotion> _motion_cache;
                mutable SliceCache _slice_cache; // Needs to be defined after _robot_state_dist_fn
                std::shared_ptr<::ompl::NearestNeighbors<SlicePtr>> _slices_nn;
                ::ompl::RNGPtr _rng;
                ::ompl::base::ValidStateSamplerPtr _state_sampler; // state sampler

            private:
                std::string _log_prefix;
            };
        }
    }
}
}
#endif //MANIPULATION_PLANNING_SUITE_RRT_H
