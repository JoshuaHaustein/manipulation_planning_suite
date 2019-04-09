//
// Created by joshua on 03/20/19.
//

#ifndef MANIPULATION_PLANNING_SUITE_OBJECT_CENTRIC_RRT_H
#define MANIPULATION_PLANNING_SUITE_OBJECT_CENTRIC_RRT_H

// MPS includes
#include <mps/planner/ompl/control/SimEnvStatePropagator.h>
#include <mps/planner/pushing/algorithm/RearrangementPlanner.h>
#include <mps/planner/pushing/oracle/OracleControlSampler.h>
// stl includes
#include <list>
#include <set>
#include <tuple>
#include <utility>

namespace mps {
namespace planner {
    namespace pushing {
        namespace algorithm {
            // Forward declarations
            class MERRTExecutionMonitor;

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
                void reset() override;
                mps::planner::ompl::planning::essentials::MotionPtr deepCopy() const override;

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
             *     that attempts to transport all movables to their respective goal configurations. 
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
                void setPushingStateEps(float eps);
                float getPushingStateEps() const;

                /**
                 *  Returns whether the given path is a valid solution for the given planning query,
                 *  assuming the world is initially in state start.
                 */
                bool isGoalPath(mps::planner::ompl::planning::essentials::PathConstPtr path,
                    const mps::planner::ompl::state::SimEnvWorldState* start,
                    RearrangementPlanner::PlanningQueryPtr pq,
                    mps::planner::ompl::planning::essentials::PathPtr updated_path)
                    override;

            protected:
                friend class MERRTExecutionMonitor;
                // high-level algorithm stubs
                virtual void sample(const PushMotionPtr& sample, PlanningBlackboard& pb);
                virtual void select(const PushMotionPtr& sample, SlicePtr& selected_slice, PlanningBlackboard& pb) const;
                virtual bool extend(const SlicePtr& current, const PushMotionPtr& target,
                    PushMotionPtr& last_motion, PlanningBlackboard& pb)
                    = 0;
                // algorithm stubs for extension function
                // helpers
                virtual void setup(PlanningQueryPtr pq, PlanningBlackboard& pb);
                SlicePtr addToTree(PushMotionPtr new_motion, PushMotionPtr parent, PlanningBlackboard& pb);
                SlicePtr getSlice(ompl::planning::essentials::MotionPtr motion, PlanningBlackboard& pb) const;
                float distanceToSlice(ompl::planning::essentials::MotionPtr motion, SlicePtr slice) const;
                /**
                 *  Sample a push: 1. sample pushing state, 2. query policy, 3. propagate
                 *  @param start_state - world state to push from
                 *  @param target_state - world state to push to (only cares about t's state)
                 *  @param approach - store pushing state in approach->getState(), if sample_pushing_state is true
                 *  @param push - store pushing action and result in
                 *  @param unsigned int t - target object
                 *  @param sample_pushing_state - if true, sample a pushing state, else try to push from robot state in
                 *                                start_state
                 *  @returns true, if valid push was sampled, else false
                 */
                bool samplePush(const ompl::state::SimEnvWorldState* start_state,
                    const ompl::state::SimEnvWorldState* target_state,
                    PushMotionPtr approach, PushMotionPtr push,
                    unsigned int t,
                    bool sample_pushing_state);
                // member variables
                const RobotStateDistanceFnPtr _robot_state_dist_fn; // distance fn within slices
                const ObjectArrangementDistanceFnPtr _slice_distance_fn; // distance fn between slices
                mps::planner::pushing::oracle::PushingOraclePtr _pushing_oracle; // pushing oracle
                mps::planner::pushing::oracle::OracleControlSamplerPtr _oracle_sampler; // wrapper around pushing oracle
                mps::planner::ompl::control::SimEnvStatePropagatorPtr _state_propagator; // physics model
                std::shared_ptr<::ompl::NearestNeighbors<SlicePtr>> _slices_nn;
                ::ompl::RNGPtr _rng;
                ::ompl::base::ValidStateSamplerPtr _state_sampler; // state sampler
                float _min_slice_distance;
                float _goal_bias;
                float _pushing_state_eps;
                mutable MotionCache<PushMotion> _motion_cache;
                mutable SliceCache _slice_cache; // Needs to be defined after _robot_state_dist_fn

            private:
                std::string _log_prefix;
            };

            typedef std::shared_ptr<MultiExtendRRT> MultiExtendRRTPtr;
            typedef std::shared_ptr<const MultiExtendRRT> MultiExtendRRTConstPtr;

            /** 
             *  Implementation of MultiExtendRRT that uses a greedy extend function.
             *  The extend function greedily pushes movables towards their goals and
             *  uses depth-limited backtracking when it fails.
             */
            class GreedyMultiExtendRRT : public MultiExtendRRT {
            public:
                GreedyMultiExtendRRT(::ompl::control::SpaceInformationPtr si,
                    mps::planner::pushing::oracle::PushingOraclePtr pushing_oracle,
                    mps::planner::pushing::oracle::RobotOraclePtr robot_oracle,
                    const std::string& robot_name);
                ~GreedyMultiExtendRRT();
                PlanningQueryPtr createPlanningQuery(mps::planner::ompl::state::goal::ObjectsRelocationGoalPtr goal_region,
                    mps::planner::ompl::state::SimEnvWorldState* start_state, const std::string& robot_name, float timeout) override;
                void setTargetTolerance(float tol); // minimal distance used in extension function -> should be smaller or equal to tolerance in goal region
                float getTargetTolerance() const;
                // void setPositionTolerance(float tol); // minimal position distance used in extension function -> should be smaller or equal to tolerance in goal region
                // float getPositionTolerance() const;
                // void setOrientationTolerance(float tol); // minimal orientation distance used in extension function -> should be smaller or equal to tolerance in goal region
                // float getOrientationTolerance() const;
                void setDisturbanceTolerance(float tol); // SE2 distance that an object may be pushed away from its goal within
                float getDisturbanceTolerance() const; // an extension step
                void setNumPushingTrials(unsigned int trials);
                unsigned int getNumPushingTrials() const;

            protected:
                // definitions for extension function
                enum class PushResult {
                    FAIL = 0,
                    POLICY_FAIL = 1,
                    MOVABLES_BLOCK_PUSH = 2,
                    PROGRESS = 3,
                    REACHED = 4
                };
                enum class ExtensionProgress {
                    FAIL = 0,
                    REACHED = 1,
                    GOAL_REACHED = 2,
                };
                typedef std::set<unsigned int> MovableSet;
                typedef std::pair<PushMotionPtr, SlicePtr> StateSlicePair;
                bool extend(const SlicePtr& current, const PushMotionPtr& target,
                    PushMotionPtr& last_motion, PlanningBlackboard& pb) override;
                virtual std::tuple<ExtensionProgress, PushMotionPtr> recursiveExtend(unsigned int t, const StateSlicePair& current,
                    MovableSet& targets, MovableSet& remainers, SliceConstPtr target_slice, PlanningBlackboard& pb);
                virtual std::tuple<PushResult, PushMotionPtr> tryPush(unsigned int t, const StateSlicePair& current,
                    MovableSet& movables, MovableSet& blockers, SliceConstPtr target_slice, bool new_push, PlanningBlackboard& pb);
                bool isCloseEnough(const PushMotionPtr& current, SliceConstPtr target_slice, unsigned int t) const;
                bool getPushBlockers(const PushMotionConstPtr& x_b, const PushMotionConstPtr& x_a,
                    const SliceConstPtr& target_slice, const MovableSet& movables,
                    MovableSet& blockers, PlanningBlackboard& pb) const;
                void computeObjectDistances(const PushMotionConstPtr& x, const SliceConstPtr& target, Eigen::VectorXf& dist_array) const;

                // parameters
                // float _orientation_tolerance; // max rot distance an object is allowed to be from its goal (within extend) to be considered reached
                // float _position_tolerance; // max trans distance an object is allowed to be from its goal (within extend) to be considered reached
                float _target_tolerance; // max trans distance an object is allowed to be from its goal (within extend) to be considered reached
                float _disturbance_tolerance; // max distance an object may be pushed away from its goal
                unsigned int _num_pushing_trials; // number of trials when trying to push an object
            private:
                const std::string _log_prefix;
                // memory cache to avoid constant reallocations
                mutable Eigen::VectorXf _vector_a;
                mutable Eigen::VectorXf _distances_before;
                mutable Eigen::VectorXf _distances_after;
            };

            class MERRTExecutionMonitor : public ExecutionMonitor {
            public:
                MERRTExecutionMonitor(MultiExtendRRTPtr planner, ExecutionCallback excall);
                MERRTExecutionMonitor(MultiExtendRRTPtr planner);
                ~MERRTExecutionMonitor() override;
                bool execute(RearrangementPlanner::PlanningQueryPtr pq) override;

                float getTransferTolerance() const;
                void setTransferTolerance(float val);

            protected:
                struct TransitSegment {
                    std::vector<PushMotionConstPtr> intended;
                    std::vector<PushMotionPtr> predicted;
                };

                struct TransferSegment {
                    std::vector<PushMotionConstPtr> intended;
                    std::vector<PushMotionPtr> predicted;
                    unsigned int target_id;
                };

                typedef std::vector<std::pair<TransitSegment, TransferSegment>> SegmentedPath;
                void segmentPath(mps::planner::ompl::planning::essentials::PathPtr intended_path,
                    mps::planner::ompl::planning::essentials::PathPtr predicted_path,
                    SegmentedPath& segments) const;
                void extractNewPath(SegmentedPath& segments, mps::planner::ompl::planning::essentials::PathPtr path) const;
                virtual bool updatePath(mps::planner::ompl::planning::essentials::PathPtr path,
                    mps::planner::ompl::state::SimEnvWorldState* state, RearrangementPlanner::PlanningQueryPtr pq);

                bool updatePathPrediction(SegmentedPath& segments, const SegmentedPath::iterator& iter,
                    RearrangementPlanner::PlanningQueryPtr pq);
                enum class PredictionUpdate {
                    INVALID = 0,
                    VALID = 1,
                    GOAL_REACHED = 2
                };
                PredictionUpdate simulateSegment(const std::vector<PushMotionConstPtr>& intended, std::vector<PushMotionPtr>& predicted,
                    PushMotionPtr& prev_motion, RearrangementPlanner::PlanningQueryPtr pq);
                virtual bool updateTransfer(const mps::planner::ompl::state::SimEnvWorldState* start,
                    TransitSegment& transit, TransferSegment& transfer, PushMotionPtr prev_motion);
                virtual std::tuple<PushMotionPtr, PushMotionPtr> tryPush(unsigned int t,
                    const mps::planner::ompl::state::SimEnvWorldState* start,
                    const mps::planner::ompl::state::SimEnvWorldState* goal);

                // const members
                const MultiExtendRRTPtr _merrt_planner;
                const ::ompl::control::SpaceInformationPtr _si;
                const mps::planner::ompl::control::SimEnvStatePropagatorPtr _propagator;
                const mps::planner::ompl::state::SimEnvWorldStateSpacePtr _state_space;
                // motion cache
                mutable MotionCache<PushMotion> _motion_cache;
                // parameters
                float _transfer_tolerance;
                unsigned int _num_pushing_trials;
                // robot specific members that change depending on the planning query
                mps::planner::ompl::state::SimEnvObjectStateSpacePtr _robot_state_space;
                unsigned int _robot_id;
            };
        }
    }
}
}
#endif //MANIPULATION_PLANNING_SUITE_RRT_H
