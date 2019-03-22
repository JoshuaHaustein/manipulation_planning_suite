//
// Created by joshua on 8/14/17.
//

#ifndef MANIPULATION_PLANNING_SUITE_RRT_H
#define MANIPULATION_PLANNING_SUITE_RRT_H

// OMPL includes
#include <ompl/base/Path.h>
#include <ompl/base/State.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/control/SpaceInformation.h>
// MPS includes
#include <mps/planner/ompl/control/SimEnvStatePropagator.h>
#include <mps/planner/pushing/algorithm/Interfaces.h>
#include <mps/planner/util/Random.h>
#include <mps/planner/util/Time.h>
#include <mps/sdf/SDF.h>
// stl
#include <mps/planner/ompl/control/NaiveControlSampler.h>
#include <mps/planner/pushing/oracle/OracleControlSampler.h>
#include <stack>

namespace mps {
namespace planner {
    namespace pushing {
        namespace algorithm {
            /**
                 * This class implements a semi-dynamic RRT algorithm for non-prehensile rearrangement.
                 * The algorithm searches for a sequence of robot actions that push a target object into some goal region.
                 * The special aspect about the semi-dynamic RRT is that the state propagator may add an
                 * additional waiting time to a robot control action to allow the environment to come to rest. After
                 * each action. This allows utilizing dynamical physical interactions between the robot and its environment
                 * without planning on full state space.
                 *
                 * This planner operates on SimEnvWorldStateSpace only as it utilizes the SimEnvStatePropagator.
                 * Also, the control space needs to be compatible to SemiDynamicVelocityControl, i.e. the controls
                 * need to be descendants of SemiDynamicVelocityControl.
                 *
                 * This class is abstract. There are several stubs that need to be implement in order for the
                 * algorithm to be working. See OracleRearrangementRRT and NaiveRearrangementRRT for fully defined
                 * implementations.
                 */
            class RearrangementRRT : public RearrangementPlanner {
            protected:
                struct PlanningBlackboard {
                    PlanningQueryPtr pq;
                    PlanningStatistics stats;
                    unsigned int robot_id;
                    explicit PlanningBlackboard(PlanningQueryPtr pq);
                };

            public:
                /**
                     * Creates a new semi-dynamic RRT algorithm. The state propagator provided in si is expected to be
                     * of type SimEnvStatePropagator. Accordingly the control space is expected to be a space of controls
                     * that inherit from SemiDynamicVelocityControl.
                     * @param si
                     */
                RearrangementRRT(::ompl::control::SpaceInformationPtr si);
                virtual ~RearrangementRRT() = 0;
                bool plan(PlanningQueryPtr pq) override;
                bool plan(PlanningQueryPtr pq, PlanningStatistics& stats) override;
                PlanningQueryPtr createPlanningQuery(mps::planner::ompl::state::goal::ObjectsRelocationGoalPtr goal_region,
                    mps::planner::ompl::state::SimEnvWorldState* start_state, const std::string& robot_name, float timeout) override;
                /**
                     *
                     * @param start
                     * @param dest
                     * @param active_obj_id
                     * @param last_motion
                     * @param pb
                     * @return
                     */
                virtual bool extend(mps::planner::ompl::planning::essentials::MotionPtr start,
                    ::ompl::base::State* dest,
                    unsigned int active_obj_id,
                    mps::planner::ompl::planning::essentials::MotionPtr& last_motion,
                    PlanningBlackboard& pb)
                    = 0;
                /**
                     * Samples a new state to extend the search tree to.
                     * @param motion - a motion which contains the new state
                     * @param target_obj_id - if the sampled state is a goal, this is the id of the target obj
                     * @return true, if the sampled state is a goal
                     */
                virtual bool sample(mps::planner::ompl::planning::essentials::MotionPtr motion,
                    unsigned int& target_obj_id,
                    PlanningBlackboard& pb);
                virtual void selectTreeNode(const ompl::planning::essentials::MotionPtr& sample,
                    ompl::planning::essentials::MotionPtr& selected_node,
                    unsigned int& active_obj_id,
                    bool sample_is_goal,
                    PlanningBlackboard& pb);
                virtual double treeDistanceFunction(const mps::planner::ompl::planning::essentials::MotionPtr& a,
                    const mps::planner::ompl::planning::essentials::MotionPtr& b) const;

                void setDebugDrawer(DebugDrawerPtr debug_drawer);
                std::shared_ptr<mps::planner::util::time::Timer> timer_ptr = std::make_shared<mps::planner::util::time::Timer>();

                // Getters and setters for parameters
                float getGoalBias() const;
                float getRobotBias() const;
                float getTargetBias() const;
                void setGoalBias(float v);
                void setTargetBias(float v);
                void setRobotBias(float v);

            protected:
                virtual void setup(PlanningQueryPtr pq, PlanningBlackboard& blackboard);
                virtual void addToTree(mps::planner::ompl::planning::essentials::MotionPtr new_motion,
                    mps::planner::ompl::planning::essentials::MotionPtr parent,
                    PlanningBlackboard& pb);
                unsigned int sampleActiveObject(const PlanningBlackboard& pb) const;
                void printState(const std::string& msg, ::ompl::base::State* state) const;

                ::ompl::control::SpaceInformationPtr _si;
                ::ompl::base::ValidStateSamplerPtr _state_sampler;
                mps::planner::ompl::state::SimEnvWorldStateSpacePtr _state_space;
                mps::planner::ompl::state::SimEnvWorldStateDistanceMeasurePtr _distance_measure;
                ::ompl::RNGPtr _rng;
                DebugDrawerPtr _debug_drawer;
                mutable MotionCache<> _motion_cache;
                // Parameters
                float _goal_bias; // fraction of times the planner should attempt to connect to a goal configuration
                float _target_bias; // fraction of times the planner should focus at least on moving the target objects
                float _robot_bias; // fraction of times the planner should focus at least on moving the robot

            private:
                void setupBlackboard(PlanningBlackboard& pb);

                std::string _log_prefix;
                std::shared_ptr<::ompl::NearestNeighbors<mps::planner::ompl::planning::essentials::MotionPtr>> _tree;
            };

            typedef std::shared_ptr<RearrangementRRT> RearrangementRRTPtr;
            typedef std::shared_ptr<const RearrangementRRT> RearrangementRRTConstPtr;
            typedef std::weak_ptr<RearrangementRRT> RearrangementRRTWeakPtr;
            typedef std::weak_ptr<const RearrangementRRT> RearrangementRRTWeakConstPtr;

            class NaiveRearrangementRRT : public RearrangementRRT {
            public:
                NaiveRearrangementRRT(::ompl::control::SpaceInformationPtr si,
                    unsigned int k = 10);
                ~NaiveRearrangementRRT() override;
                void setup(PlanningQueryPtr pq, PlanningBlackboard& blackboard) override;
                PlanningQueryPtr createPlanningQuery(mps::planner::ompl::state::goal::ObjectsRelocationGoalPtr goal_region,
                    mps::planner::ompl::state::SimEnvWorldState* start_state, const std::string& robot_name, float timeout) override;
                bool extend(mps::planner::ompl::planning::essentials::MotionPtr start,
                    ::ompl::base::State* dest,
                    unsigned int active_obj_id,
                    mps::planner::ompl::planning::essentials::MotionPtr& last_motion,
                    PlanningBlackboard& pb) override;

                unsigned int getNumControlSamples() const;
                void setNumControlSamples(unsigned int samples);

            protected:
                unsigned int _num_control_samples; // number of control samples
            private:
                mps::planner::ompl::control::NaiveControlSampler _control_sampler;
            };
            typedef std::shared_ptr<NaiveRearrangementRRT> NaiveRearrangementRRTPtr;
            typedef std::shared_ptr<const NaiveRearrangementRRT> NaiveRearrangementRRTConstPtr;
            typedef std::weak_ptr<NaiveRearrangementRRT> NaiveRearrangementRRTWeakPtr;
            typedef std::weak_ptr<const NaiveRearrangementRRT> NaiveRearrangementRRTWeakConstPtr;

            class HybridActionRRT : public RearrangementRRT {
            public:
                HybridActionRRT(::ompl::control::SpaceInformationPtr si,
                    mps::planner::pushing::oracle::PushingOraclePtr pushing_oracle,
                    mps::planner::pushing::oracle::RobotOraclePtr robot_oracle,
                    const std::string& robot_name);
                ~HybridActionRRT();
                PlanningQueryPtr createPlanningQuery(mps::planner::ompl::state::goal::ObjectsRelocationGoalPtr goal_region,
                    mps::planner::ompl::state::SimEnvWorldState* start_state, const std::string& robot_name, float timeout) override;
                bool extend(mps::planner::ompl::planning::essentials::MotionPtr start,
                    ::ompl::base::State* dest,
                    unsigned int active_obj_id,
                    mps::planner::ompl::planning::essentials::MotionPtr& last_motion,
                    PlanningBlackboard& pb) override;
                unsigned int getNumControlSamples() const;
                void setNumControlSamples(unsigned int samples);
                float getActionRandomness() const;
                void setActionRandomness(float ar);

            protected:
                unsigned int _num_control_samples; // number of control samples
                float _action_randomness; // parameter in [0, 1] that determines randomness of action sampling (prand)

            private:
                void sampleActionSequence(std::vector<::ompl::control::Control const*>& controls,
                    mps::planner::ompl::planning::essentials::MotionPtr start,
                    ::ompl::base::State* dest,
                    PlanningBlackboard& pb);
                void forwardPropagateActionSequence(const std::vector<::ompl::control::Control const*>& controls,
                    mps::planner::ompl::planning::essentials::MotionPtr start,
                    std::vector<mps::planner::ompl::planning::essentials::MotionPtr>& state_action_seq,
                    PlanningBlackboard& pb);
                void freeMotionList(std::vector<mps::planner::ompl::planning::essentials::MotionPtr>& motions);
                mps::planner::ompl::control::SimEnvStatePropagatorPtr _state_propagator;
                mps::planner::pushing::oracle::OracleControlSamplerPtr _oracle_sampler;
            };
            typedef std::shared_ptr<HybridActionRRT> HybridActionRRTPtr;
            typedef std::shared_ptr<const HybridActionRRT> HybridActionRRTConstPtr;
            typedef std::weak_ptr<HybridActionRRT> HybridActionRRTWeakPtr;
            typedef std::weak_ptr<const HybridActionRRT> HybridActionRRTWeakConstPtr;

            class OracleRearrangementRRT : public RearrangementRRT {
            public:
                OracleRearrangementRRT(::ompl::control::SpaceInformationPtr si,
                    mps::planner::pushing::oracle::PushingOraclePtr pushing_oracle,
                    mps::planner::pushing::oracle::RobotOraclePtr robot_oracle,
                    const std::string& robot_name);
                ~OracleRearrangementRRT() override;
                PlanningQueryPtr createPlanningQuery(mps::planner::ompl::state::goal::ObjectsRelocationGoalPtr goal_region,
                    mps::planner::ompl::state::SimEnvWorldState* start_state, const std::string& robot_name, float timeout) override;
                void selectTreeNode(const ompl::planning::essentials::MotionPtr& sample,
                    ompl::planning::essentials::MotionPtr& selected_node,
                    unsigned int& active_obj_id,
                    bool sample_is_goal,
                    PlanningBlackboard& pb) override;
                bool extend(mps::planner::ompl::planning::essentials::MotionPtr start,
                    ::ompl::base::State* dest,
                    unsigned int active_obj_id,
                    mps::planner::ompl::planning::essentials::MotionPtr& last_motion,
                    PlanningBlackboard& pb) override;

                float getActionRandomness() const;
                void setActionRandomness(float ar);
                float getStateNoise() const;
                void setStateNoise(float sn);
                /**
                     * Returns the oracle sampler used by this planner.
                     * This function is mostly here for debug purposes allowing a developer
                     * to study the behaviour of the oracle sampler.
                     * */
                mps::planner::pushing::oracle::OracleControlSamplerPtr getOracleSampler() const;

            protected:
                void extendStep(const std::vector<const ::ompl::control::Control*>& controls,
                    const mps::planner::ompl::planning::essentials::MotionPtr& start_motion,
                    mps::planner::ompl::planning::essentials::MotionPtr& result_motion,
                    PlanningBlackboard& pb,
                    bool& extension_success,
                    bool& goal_reached);

                float _action_randomness; // parameter in [0, 1] that determines randomness of action sampling (prand)
                float _feasible_state_noise; // probability (in [0,1]) to sample a feasible state uniformly rather than from the oracle
                mps::planner::ompl::control::SimEnvStatePropagatorPtr _state_propagator;
                mps::planner::pushing::oracle::OracleControlSamplerPtr _oracle_sampler;
            };

            class SliceBasedOracleRRT : public OracleRearrangementRRT {
            public:
                SliceBasedOracleRRT(::ompl::control::SpaceInformationPtr si,
                    mps::planner::pushing::oracle::PushingOraclePtr pushing_oracle,
                    mps::planner::pushing::oracle::RobotOraclePtr robot_oracle,
                    const std::string& robot_name);
                ~SliceBasedOracleRRT() override;
                void setup(PlanningQueryPtr pq, PlanningBlackboard& blackboard) override;
                void selectTreeNode(const ompl::planning::essentials::MotionPtr& sample,
                    ompl::planning::essentials::MotionPtr& selected_node,
                    unsigned int& active_obj_id,
                    bool sample_is_goal,
                    PlanningBlackboard& pb) override;
                void addToTree(mps::planner::ompl::planning::essentials::MotionPtr new_motion,
                    mps::planner::ompl::planning::essentials::MotionPtr parent,
                    PlanningBlackboard& pb) override;

            protected:
                typedef std::tuple<SlicePtr, float> ExtensionCandidateTuple;
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
                float feasible_state_noise; // probability (in [0,1]) to sample a feasible state uniformly rather than from the oracle
                mutable SliceCache _slice_cache; // Needs to be defined after _robot_state_dist_fn
            };

            //////////////////////////////////////////////////////////////////////////////////////
            /////////////////////////////// Shortcutting algorithms //////////////////////////////
            //////////////////////////////////////////////////////////////////////////////////////
            class Shortcutter {
                /***
                     * A shortcutter takes a path found by a RearrangementRRT and shortcuts it, i.e. it tries 
                     * to remove unneccessary actions from the solutions.
                     */
            public:
                struct ShortcutQuery {
                    ompl::state::goal::ObjectsRelocationGoalPtr goal_region; // goal region
                    ompl::planning::essentials::CostFunctionPtr cost_function; // objective for shortcutting
                    std::string robot_name;
                    std::function<bool()> stopping_condition; // optionally a customized stopping condition
                    float cost_before_shortcut;
                    float cost_after_shortcut;
                    ShortcutQuery(ompl::state::goal::ObjectsRelocationGoalPtr goal_region_in,
                        ompl::planning::essentials::CostFunctionPtr cost_function_in,
                        std::string robot_name_in)
                    {
                        stopping_condition = []() { return false; };
                        goal_region = goal_region_in;
                        cost_function = cost_function_in;
                        robot_name = robot_name_in;
                    }
                };
                Shortcutter(::ompl::control::SpaceInformationPtr si);
                virtual ~Shortcutter() = 0;

                void setDebugDrawer(mps::planner::pushing::algorithm::DebugDrawerPtr debug_drawer);

                /**
                         *  Shortcut the given path.
                         * @param path - a path created by a rearrangement RRT. If the path is invalid,
                         *               the path remains unchanged, else the path is shortcut.
                         * @param pq - shortcut query
                         * @max_time - maximal duration this function is allowed to run (in seconds)
                         */
                virtual void shortcut(mps::planner::ompl::planning::essentials::PathPtr path,
                    ShortcutQuery& pq,
                    float max_time)
                    = 0;
                virtual std::string getName() const = 0;

            protected:
                ::ompl::control::SpaceInformationPtr _si;
                mps::planner::ompl::control::SimEnvStatePropagatorPtr _state_propagator;
                mps::planner::util::time::Timer _timer;
                MotionCache<> _motion_cache;
                // Methods
                mps::planner::ompl::planning::essentials::PathPtr getNewPath();
                // caches the given path, if clear_id >= 0, the motions with id >= clear_id are also cached
                void cachePath(mps::planner::ompl::planning::essentials::PathPtr ptr, int clear_id = -1);
                /***
                         *  Forward propagates the actions stored in new_motions starting from the last state stored in path.
                         *  All successive states are added to path.  Aborts execution if any intermediate state is invalid or a goal
                         *  Returns pair <b_success, b_goal>, where b_success is true iff all propagations were valid
                         *      and b_goal is true iff path->last() is a goal.
                         */
                std::pair<bool, bool> forwardPropagatePath(mps::planner::ompl::planning::essentials::PathPtr path,
                    std::vector<mps::planner::ompl::planning::essentials::MotionPtr>& new_motions,
                    ShortcutQuery& sq);
                std::pair<bool, bool> forwardPropagatePath(mps::planner::ompl::planning::essentials::PathPtr path,
                    const std::vector<const ::ompl::control::Control*>& controls,
                    ShortcutQuery& sq);
                /***
                         *  Forward propagates the actions stored in old_path starting from the state
                         *  stored with id old_path_continuation.
                         *  All successive states are added to path.  
                         *  Aborts execution if any intermediate state is invalid or a goal
                         *  Returns pair <b_success, b_goal>, where b_success is true iff all propagations were valid
                         *      and b_goal is true iff path->last() is a goal.
                         */
                std::pair<bool, bool> forwardPropagatePath(mps::planner::ompl::planning::essentials::PathPtr path,
                    mps::planner::ompl::planning::essentials::PathPtr old_path,
                    unsigned int old_path_continuation,
                    ShortcutQuery& sq);
                /***
                         * Forward propagates the action stored in new_wp and from there on all actions
                         * in old_path starting at index old_path_continuation. The resulting states
                         * are added as new motions to path. The propagation starts from the last state stored
                         * in path.
                         *  Returns pair <b_success, b_goal>, where b_success is true iff all propagations were valid
                         *      and b_goal is true iff path->last() is a goal.
                         */
                std::pair<bool, bool> forwardPropagatePath(mps::planner::ompl::planning::essentials::PathPtr path,
                    std::vector<mps::planner::ompl::planning::essentials::MotionPtr>& new_motions,
                    mps::planner::ompl::planning::essentials::PathPtr old_path,
                    unsigned int old_path_continuation,
                    ShortcutQuery& sq);
                void showState(::ompl::base::State* state, const std::string& msg);
                mps::planner::pushing::algorithm::DebugDrawerPtr _debug_drawer;

            private:
                std::stack<mps::planner::ompl::planning::essentials::PathPtr> _path_cache;
            };
            typedef std::shared_ptr<Shortcutter> ShortcutterPtr;
            typedef std::shared_ptr<const Shortcutter> ShortcutterConstPtr;
            typedef std::weak_ptr<Shortcutter> ShortcutterWeakPtr;
            typedef std::weak_ptr<const Shortcutter> ShortcutterWeakConstPtr;

            class NaiveShortcutter : public Shortcutter, public std::enable_shared_from_this<NaiveShortcutter> {
                /**
                     * A NaiveShortCutter attempts to shortcut a path by steering the robot between two randomly 
                     * sampled states (x_i, x_j), j > i + 1, and checking whether the subsequent actions from 
                     * state x_j still lead to success.
                     */
            public:
                NaiveShortcutter(::ompl::control::SpaceInformationPtr si,
                    mps::planner::pushing::oracle::RobotOraclePtr robot_oracle);
                virtual ~NaiveShortcutter();
                void shortcut(mps::planner::ompl::planning::essentials::PathPtr path,
                    ShortcutQuery& pq,
                    float max_time) override;
                std::string getName() const override;

            protected:
                void computeRobotActions(std::vector<mps::planner::ompl::planning::essentials::MotionPtr>& motions,
                    ::ompl::base::State* start_state,
                    ::ompl::base::State* end_state,
                    unsigned int robot_id);
                // creates all pairs of waypoints
                void createAllPairs(std::vector<std::pair<unsigned int, unsigned int>>& all_pairs, unsigned int n) const;

            private:
                mps::planner::pushing::oracle::RobotOraclePtr _robot_oracle;
            };
            typedef std::shared_ptr<NaiveShortcutter> NaiveShortCutterPtr;
            typedef std::shared_ptr<const NaiveShortcutter> NaiveShortCutterConstPtr;
            typedef std::weak_ptr<NaiveShortcutter> NaiveShortCutterWeakPtr;
            typedef std::weak_ptr<const NaiveShortcutter> NaiveShortCutterWeakConstPtr;

            class LocalShortcutter : public Shortcutter {
            public:
                LocalShortcutter(::ompl::control::SpaceInformationPtr si,
                    mps::planner::pushing::oracle::RobotOraclePtr robot_oracle,
                    const std::string& robot_name);
                virtual ~LocalShortcutter();
                void shortcut(mps::planner::ompl::planning::essentials::PathPtr path,
                    ShortcutQuery& pq,
                    float max_time) override;
                std::string getName() const override;

            protected:
                /** Computes a shortcut from the state in prefix_path->last() to end_state and extends prefix_path
                         * with the respective motions.
                         * @return boolean pair (extension_success, path->last() is a goal)
                        */
                virtual std::pair<bool, bool> computeShortcut(mps::planner::ompl::planning::essentials::PathPtr prefix_path,
                    ::ompl::base::State* end_state, ShortcutQuery& pq);
                void computeRobotActions(std::vector<mps::planner::ompl::planning::essentials::MotionPtr>& motions,
                    ::ompl::base::State* start_state,
                    ::ompl::base::State* end_state);
                unsigned int _robot_id;

            private:
                mps::planner::pushing::oracle::RobotOraclePtr _robot_oracle;
            };

            class LocalOracleShortcutter : public LocalShortcutter {
            public:
                LocalOracleShortcutter(::ompl::control::SpaceInformationPtr si,
                    mps::planner::pushing::oracle::RobotOraclePtr robot_oracle,
                    mps::planner::pushing::oracle::PushingOraclePtr pushing_oracle,
                    const std::string& robot_name);
                virtual ~LocalOracleShortcutter();
                std::string getName() const override;

            protected:
                // computes which object the shortcutting between start_state and end_state should focus on.
                // both states are assumend to be SimEnvWorldStates
                unsigned int selectObject(::ompl::base::State* start_state, ::ompl::base::State* end_state);
                std::pair<bool, bool> computeShortcut(mps::planner::ompl::planning::essentials::PathPtr prefix_path,
                    ::ompl::base::State* end_state, ShortcutQuery& pq) override;

            private:
                mps::planner::pushing::oracle::OracleControlSamplerPtr _oracle_sampler;
            };

            typedef std::shared_ptr<LocalOracleShortcutter> LocalOracleShortcutterPtr;
            typedef std::shared_ptr<const LocalOracleShortcutter> LocalOracleShortcutterConstPtr;
            typedef std::weak_ptr<LocalOracleShortcutter> LocalOracleShortcutterWeakPtr;
            typedef std::weak_ptr<const LocalOracleShortcutter> LocalOracleShortcutterWeakConstPtr;

            class OracleShortcutter : public Shortcutter, public std::enable_shared_from_this<OracleShortcutter> {
                /**
                     * A OracleShortcutter attempts to shortcut a path by using the oracle to move between two randomly 
                     * sampled states (x_i, x_j), j > i + 1, and checking whether the subsequent actions from 
                     * state x_j still lead to success. In contrast to the NaiveShortCutter, this shortcutter attempts
                     * to push an object towards its respective state in x_j before steering the robot to its state.
                     */
            public:
                OracleShortcutter(::ompl::control::SpaceInformationPtr si,
                    mps::planner::pushing::oracle::RobotOraclePtr robot_oracle,
                    mps::planner::pushing::oracle::PushingOraclePtr pushing_oracle,
                    const std::string& robot_name);
                virtual ~OracleShortcutter();
                void shortcut(mps::planner::ompl::planning::essentials::PathPtr path,
                    ShortcutQuery& pq,
                    float max_time) override;
                std::string getName() const override;

            protected:
                typedef std::deque<std::pair<unsigned int, unsigned int>> PairQueue;
                typedef std::vector<unsigned int> ObjectIds;
                typedef std::unordered_map<std::pair<unsigned int, unsigned int>, ObjectIds, boost::hash<std::pair<unsigned int, unsigned int>>> PairMap;

            private:
                mps::planner::pushing::oracle::OracleControlSamplerPtr _oracle_sampler;
                // Selects which object to push for shortcutting and updates pair data structures
                unsigned int selectObject(mps::planner::ompl::planning::essentials::PathPtr current_path,
                    std::pair<unsigned int, unsigned int>& current_pair,
                    PairMap& pair_to_objects, PairQueue& pair_queue,
                    unsigned int robot_id) const;
                // computes the actual shortcut and extends new_path
                bool computeShortcut(mps::planner::ompl::planning::essentials::PathPtr new_path,
                    mps::planner::ompl::planning::essentials::PathPtr current_path,
                    unsigned int target_id, unsigned int robot_id, unsigned int object_id,
                    ShortcutQuery& sq);
                void fillPairQueue(PairQueue& all_pairs, unsigned int n) const;
                bool extendPath(mps::planner::ompl::planning::essentials::PathPtr path,
                    const std::vector<const ::ompl::control::Control*>& controls,
                    bool& goal_reached,
                    ShortcutQuery& sq);
            };
            typedef std::shared_ptr<OracleShortcutter> OracleShortcutterPtr;
            typedef std::shared_ptr<const OracleShortcutter> OracleShortcutterConstPtr;
            typedef std::weak_ptr<OracleShortcutter> OracleShortcutterWeakPtr;
            typedef std::weak_ptr<const OracleShortcutter> OracleShortcutterWeakConstPtr;
        }
    }
}
}
#endif //MANIPULATION_PLANNING_SUITE_RRT_H
