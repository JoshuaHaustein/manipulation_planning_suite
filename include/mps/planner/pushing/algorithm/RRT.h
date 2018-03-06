//
// Created by joshua on 8/14/17.
//

#ifndef MANIPULATION_PLANNING_SUITE_RRT_H
#define MANIPULATION_PLANNING_SUITE_RRT_H

// OMPL includes
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/State.h>
#include <ompl/base/Path.h>
#include <ompl/datastructures/NearestNeighbors.h>
// MPS includes
#include <mps/planner/ompl/control/SimEnvStatePropagator.h>
#include <mps/planner/ompl/planning/Essentials.h>
#include <mps/planner/ompl/state/SimEnvWorldStateDistanceMeasure.h>
#include <mps/planner/ompl/state/goal/ObjectsRelocationGoal.h>
#include <mps/planner/util/Random.h>
#include <mps/planner/util/Time.h>
#include <mps/sdf/SDF.h>
// stl
#include <stack>
#include <mps/planner/ompl/control/NaiveControlSampler.h>
#include <mps/planner/pushing/oracle/OracleControlSampler.h>
#include <iomanip>

namespace mps {
    namespace planner {
        namespace pushing {
            namespace algorithm {
                class DebugDrawer;
                typedef std::shared_ptr<DebugDrawer> DebugDrawerPtr;
                typedef std::weak_ptr<DebugDrawer> DebugDrawerWeakPtr;
                class SliceDrawerInterface;
                typedef std::shared_ptr<SliceDrawerInterface> SliceDrawerInterfacePtr;

                struct PlanningStatistics {
                    unsigned int num_iterations;
                    unsigned int num_state_propagations;
                    unsigned int num_samples;
                    unsigned int num_nearest_neighbor_queries;
                    float runtime;
                    bool success;
                    float cost_before_shortcut;
                    float cost_after_shortcut;
                    bool reproducible;
                    bool reproducible_after_shortcut;

                    void print(std::ostream& os) const{
                        os << "Planning statistics: \n";
                        os << "     num_iterations: " << num_iterations;
                        os << "     num_state_propagations: " << num_state_propagations;
                        os << "     num_samples: " << num_samples;
                        os << "     num_nearest_neighbor_queries: " << num_nearest_neighbor_queries;
                        os << "     run_time: " << std::fixed << runtime << std::setprecision(3);
                        os << "     success: " << success;
                        os << "     reproducible: " << reproducible;
                        os << "     cost_before_shortcut: " << cost_before_shortcut << std::setprecision(3);
                        os << "     cost_after_shortcut: " << cost_after_shortcut << std::setprecision(3);
                        os << "     reproducible_after_shortcut: " << reproducible_after_shortcut;
                        os << std::endl;
                    }

                    void printCVSHeader(std::ostream& os) const {
                        os << "#iterations,#propagations,#samples,#nn_queries,runtime,success,reproducible,cost_before_shortcut,cost_after_shortcut,reproducible_after_shortcut" << std::endl;
                    }

                    void printCVS(std::ostream& os) const {
                        os << num_iterations << ", "
                           << num_state_propagations << ", "
                           << num_samples << ", "
                           << num_nearest_neighbor_queries << ", "
                           << runtime << ", "
                           << success << ", "
                           << reproducible << ", " 
                           << cost_before_shortcut << ", " 
                           << cost_after_shortcut << ", " 
                           << reproducible_after_shortcut << std::endl;
                    }

                    std::string to_string() const{
                        std::stringstream ss;
                        print(ss);
                        return ss.str();
                    }

                    PlanningStatistics() {
                        num_nearest_neighbor_queries = 0;
                        num_iterations = 0;
                        num_state_propagations = 0;
                        num_samples = 0;
                        runtime = 0.0;
                        success = false;
                        reproducible = false;
                        cost_before_shortcut = 0.0f;
                        cost_after_shortcut = 0.0f;
                        reproducible_after_shortcut = false;
                    }
                };

                // TODO we could inherit from ::ompl::base::Planner or could define our own planner interface, if needed
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
                 * need to be descendents of SemiDynamicVelocityControl.
                 *
                 * This class is abstract. There are several stubs that need to be implement in order for the
                 * algorithm to be working. See OracleRearrangementRRT and NaiveRearrangementRRT for fully defined
                 * implementations.
                 */
                class RearrangementRRT {
                public:

                    struct PlanningQuery {
                        ompl::state::goal::ObjectsRelocationGoalPtr goal_region; // goal region
                        ::ompl::base::State* start_state; // start state of the problem (SimEnvWorldStateSpace)
                        std::string robot_name;
                        float time_out; // time out in seconds
                        float goal_bias; // fraction of times the planner should attempt to connect to a goal configuration
                        float target_bias; // fraction of times the planner should focus at least on moving the target objects
                        float robot_bias; // fraction of times the planner should focus at least on moving the robot
                        float min_state_distance; // minimum radius for Voronoi distance for an object/robot state // TODO technically we would need a different value for each uniqie state space -> should maybe move this into state space
                        float min_slice_distance; // not set by user, is updated using min_state_distance
                        bool do_slice_ball_projection; // if true, SliceBasedOracleRRT projects sample slices to a ball with radius max_slice_distance before attempting an extension
                        float max_slice_distance; // maximum distance two slices can be apart from each other such that there still can exist an action connecting them
                        float action_randomness; // parameter in [0, 1] that determines randomness of action sampling (prand)
                        float feasible_state_noise; // probability (in [0,1]) to sample a feasible state uniformly rather than from the oracle
                        unsigned int num_control_samples; // number of control samples (only relevant for some algorithms)
                        std::vector<float> weights; // optional weights for the distance function
                        std::function<bool()> stopping_condition; // optionally a customized stopping condition
                        PlanningQuery(ompl::state::goal::ObjectsRelocationGoalPtr goal_region,
                                      ::ompl::base::State* start_state,
                                      float time_out,
                                      const std::string& robot_name);
                        PlanningQuery(const PlanningQuery& other);
                        std::string toString() const;
                    };


                protected:
                    struct PlanningBlackboard {
                        PlanningQuery pq;
                        PlanningStatistics stats;
                        unsigned int robot_id;
                        explicit PlanningBlackboard(PlanningQuery pq);
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
                    bool plan(const PlanningQuery& pq,
                              mps::planner::ompl::planning::essentials::PathPtr path);
                    virtual bool plan(const PlanningQuery& pq,
                                      mps::planner::ompl::planning::essentials::PathPtr path,
                                      PlanningStatistics& stats);
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
                                        PlanningBlackboard& pb) = 0;
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
                protected:
                    virtual void setup(const PlanningQuery& pq, PlanningBlackboard& blackboard);
                    virtual void addToTree(mps::planner::ompl::planning::essentials::MotionPtr new_motion,
                                           mps::planner::ompl::planning::essentials::MotionPtr parent,
                                           PlanningBlackboard& pb);
                    unsigned int sampleActiveObject(const PlanningBlackboard& pb) const;
                    void printState(const std::string& msg, ::ompl::base::State* state) const;
                    mps::planner::ompl::planning::essentials::MotionPtr getNewMotion();
                    void cacheMotion(mps::planner::ompl::planning::essentials::MotionPtr ptr);

                    ::ompl::control::SpaceInformationPtr _si;
                    ::ompl::base::ValidStateSamplerPtr _state_sampler;
                    mps::planner::ompl::state::SimEnvWorldStateSpacePtr _state_space;
                    mps::planner::ompl::state::SimEnvWorldStateDistanceMeasurePtr _distance_measure;
                    ::ompl::RNGPtr _rng;
                    DebugDrawerPtr _debug_drawer;

                private:
                    void setupBlackboard(PlanningBlackboard& pb);

                    std::string _log_prefix;
                    std::shared_ptr<::ompl::NearestNeighbors< mps::planner::ompl::planning::essentials::MotionPtr > > _tree;
                    std::stack<mps::planner::ompl::planning::essentials::MotionPtr> _motions_cache;
                };

                typedef std::shared_ptr<RearrangementRRT> RearrangementRRTPtr;
                typedef std::shared_ptr<const RearrangementRRT> RearrangementRRTConstPtr;
                typedef std::weak_ptr<RearrangementRRT> RearrangementRRTWeakPtr;
                typedef std::weak_ptr<const RearrangementRRT> RearrangementRRTWeakConstPtr;

                class NaiveRearrangementRRT : public RearrangementRRT {
                public:
                    NaiveRearrangementRRT(::ompl::control::SpaceInformationPtr si,
                                          unsigned int k=10);
                    ~NaiveRearrangementRRT() override;
                    void setup(const PlanningQuery& pq, PlanningBlackboard& blackboard) override;
                    bool extend(mps::planner::ompl::planning::essentials::MotionPtr start,
                                ::ompl::base::State* dest,
                                unsigned int active_obj_id,
                                mps::planner::ompl::planning::essentials::MotionPtr& last_motion,
                                PlanningBlackboard& pb) override;

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
                        bool extend(mps::planner::ompl::planning::essentials::MotionPtr start,
                                    ::ompl::base::State* dest,
                                    unsigned int active_obj_id,
                                    mps::planner::ompl::planning::essentials::MotionPtr& last_motion,
                                    PlanningBlackboard& pb) override;
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
                    void setup(const PlanningQuery& pq, PlanningBlackboard& blackboard) override;
                    void selectTreeNode(const ompl::planning::essentials::MotionPtr& sample,
                                        ompl::planning::essentials::MotionPtr& selected_node,
                                        unsigned int& active_obj_id,
                                        bool sample_is_goal,
                                        PlanningBlackboard& pb) override;
                    void addToTree(mps::planner::ompl::planning::essentials::MotionPtr new_motion,
                                   mps::planner::ompl::planning::essentials::MotionPtr parent,
                                   PlanningBlackboard& pb) override;

                    struct Slice {
                        typedef std::function<double(const mps::planner::ompl::planning::essentials::MotionPtr&,
                                                     const mps::planner::ompl::planning::essentials::MotionPtr&)> SliceDistanceFn;
                        Slice(mps::planner::ompl::planning::essentials::MotionPtr repr,
                              SliceDistanceFn distance_fn);
                        ~Slice();
                        void addSample(ompl::planning::essentials::MotionPtr motion);
                        void clear();
                        void reset(ompl::planning::essentials::MotionPtr repr);
                        std::shared_ptr<::ompl::NearestNeighbors<mps::planner::ompl::planning::essentials::MotionPtr> > slice_samples_nn;
                        std::vector<ompl::planning::essentials::MotionPtr> slice_samples_list;
                        mps::planner::ompl::planning::essentials::MotionPtr repr; // representative
                    };
                    typedef std::shared_ptr<Slice> SlicePtr;
                    typedef std::shared_ptr<const Slice> SliceConstPtr;

                protected:
                    // type definitions
                    struct WithinSliceDistance {
                        ompl::state::SimEnvWorldStateDistanceMeasure distance_measure;
                        double distance(const ompl::planning::essentials::MotionPtr& motion_a,
                                        const ompl::planning::essentials::MotionPtr& motion_b) const;
                        void setRobotId(unsigned int id);
                        explicit WithinSliceDistance(ompl::state::SimEnvWorldStateSpacePtr state_space,
                                            const std::vector<float>& weights=std::vector<float>());
                    };

                    struct SliceDistance {
                        ompl::state::SimEnvWorldStateDistanceMeasure distance_measure;
                        void setRobotId(unsigned int id);
                        double distance(const SliceConstPtr& slice_a, const SliceConstPtr& slice_b) const;
                        explicit SliceDistance(ompl::state::SimEnvWorldStateSpacePtr state_space,
                                               const std::vector<float>& weights=std::vector<float>());
                    };

                    typedef std::tuple<SlicePtr, float> ExtensionCandidateTuple;
                    // member functions
                    SlicePtr getSlice(ompl::planning::essentials::MotionPtr motion, PlanningBlackboard& pb) const;
                    float distanceToSlice(ompl::planning::essentials::MotionPtr motion, SlicePtr slice) const;
                    void projectSliceOnBall(SlicePtr sample_slice,
                                            SliceConstPtr center_slice,
                                            float radius,
                                            PlanningBlackboard& pb);

                    SlicePtr getNewSlice(ompl::planning::essentials::MotionPtr motion) const;
                    void cacheSlice(SlicePtr slice) const;
                    ExtensionCandidateTuple selectCandidateSlice(const std::vector<ExtensionCandidateTuple>& candidates) const;
                    // member variables
                    std::vector<SlicePtr> _slices_list;
                    std::shared_ptr<::ompl::NearestNeighbors<SlicePtr> > _slices_nn;
                    ::ompl::base::StateSamplerPtr _robot_state_sampler;
                    ::ompl::base::StateSpacePtr _robot_state_space;
                    WithinSliceDistance _within_slice_distance_fn;
                    SliceDistance _slice_distance_fn;
                    mps::planner::pushing::oracle::PushingOraclePtr _pushing_oracle;
                private:
                    mutable std::stack<SlicePtr> _slices_cache;
                };

                // TODO this is future work
                // class GNATSamplingSliceBasedOracleRRT : public SliceBasedOracleRRT {
                // public:
                //     GNATSamplingSliceBasedOracleRRT(::ompl::control::SpaceInformationPtr si,
                //                                 mps::planner::pushing::oracle::PushingOraclePtr pushing_oracle,
                //                                 mps::planner::pushing::oracle::RobotOraclePtr robot_oracle,
                //                                 const std::string& robot_name,
                //                                 const oracle::OracleControlSampler::Parameters& params=
                //                                 oracle::OracleControlSampler::Parameters());
                //     ~GNATSamplingSliceBasedOracleRRT() override;
                //     bool sample(mps::planner::ompl::planning::essentials::MotionPtr motion,
                //                 unsigned int& target_obj_id,
                //                 PlanningBlackboard& pb) override;
                // protected:
                // private:
                // };

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
                                stopping_condition = []() {return false;};
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
                                              float max_time) = 0;
                        virtual std::string getName() const = 0;
                    protected:
                        ::ompl::control::SpaceInformationPtr _si;
                        mps::planner::ompl::control::SimEnvStatePropagatorPtr _state_propagator;
                        mps::planner::util::time::Timer _timer;
                        // Methods
                        mps::planner::ompl::planning::essentials::MotionPtr getNewMotion();
                        void cacheMotion(mps::planner::ompl::planning::essentials::MotionPtr ptr);
                        void cacheMotions(std::vector<mps::planner::ompl::planning::essentials::MotionPtr>& motions);
                        mps::planner::ompl::planning::essentials::PathPtr getNewPath();
                        // caches the given path, if clear_id >= 0, the motions with id >= clear_id are also cached
                        void cachePath(mps::planner::ompl::planning::essentials::PathPtr ptr, int clear_id=-1);
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
                        std::stack<mps::planner::ompl::planning::essentials::MotionPtr> _motions_cache;
                        std::stack<mps::planner::ompl::planning::essentials::PathPtr> _path_cache;
                };
                typedef std::shared_ptr<Shortcutter> ShortcutterPtr;
                typedef std::shared_ptr<const Shortcutter> ShortcutterConstPtr;
                typedef std::weak_ptr<Shortcutter> ShortcutterWeakPtr;
                typedef std::weak_ptr<const Shortcutter> ShortcutterWeakConstPtr;

                class NaiveShortcutter : public Shortcutter, public std::enable_shared_from_this<NaiveShortcutter>  {
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
                        void createAllPairs(std::vector<std::pair<unsigned int, unsigned int> >& all_pairs, unsigned int n) const;
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

                class LocalOracleShortcutter: public LocalShortcutter {
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

                class OracleShortcutter : public Shortcutter, public std::enable_shared_from_this<OracleShortcutter>  {
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
                        typedef std::deque<std::pair<unsigned int, unsigned int> > PairQueue;
                        typedef std::vector<unsigned int> ObjectIds;
                        typedef std::unordered_map<std::pair<unsigned int, unsigned int>, ObjectIds, boost::hash<std::pair<unsigned int, unsigned int> > > PairMap;
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


                class DebugDrawer : public std::enable_shared_from_this<DebugDrawer> {
                    // TODO this class may be overfit to a 2d planning case.
                public:
                    DebugDrawer(sim_env::WorldViewerPtr world, unsigned int robot_id, const std::vector<unsigned int>& target_indices);
                    DebugDrawer(sim_env::WorldViewerPtr world, SliceDrawerInterfacePtr slice_drawer, unsigned int robot_id, const std::vector<unsigned int>& target_indices);
                    ~DebugDrawer();
                    void setSliceDrawer(SliceDrawerInterfacePtr slice_drawer);
                    void setRobotId(unsigned int robot_id);
                    void setTargetIds(const std::vector<unsigned int>& target_ids);
                    void addNewMotion(mps::planner::ompl::planning::essentials::MotionPtr motion);
                    void clear(bool clear_slice_drawer=true);
                    void drawStateTransition(const ompl::state::SimEnvObjectState* parent_state,
                                             const ompl::state::SimEnvObjectState* new_state,
                                             const Eigen::Vector4f& color);
                    void showState(const ompl::state::SimEnvWorldState* state, const ompl::state::SimEnvWorldStateSpaceConstPtr state_space);
                    void addNewSlice(mps::planner::pushing::algorithm::SliceBasedOracleRRT::SliceConstPtr slice);
                    SliceDrawerInterfacePtr getSliceDrawer();

                private:
                    sim_env::WorldViewerPtr _world_viewer;
                    SliceDrawerInterfacePtr _slice_drawer;
                    std::vector<sim_env::WorldViewer::Handle> _handles;
                    unsigned int _robot_id;
                    std::vector<unsigned int> _target_ids;
                };

                class SliceDrawerInterface {
                public:
                    virtual ~SliceDrawerInterface() = 0;
                    // TODO functions needed to draw a slice
                    virtual void clear() = 0;
                    virtual void addSlice(mps::planner::pushing::algorithm::SliceBasedOracleRRT::SliceConstPtr slice) = 0;
                    void setDebugDrawer(mps::planner::pushing::algorithm::DebugDrawerPtr debug_drawer);
                    void setStateSpace(mps::planner::ompl::state::SimEnvWorldStateSpacePtr state_space);
                protected:
                    mps::planner::ompl::state::SimEnvWorldStateSpacePtr _state_space;
                    mps::planner::pushing::algorithm::DebugDrawerWeakPtr _debug_drawer;
                };
            }
        }
    }
}
#endif //MANIPULATION_PLANNING_SUITE_RRT_H
