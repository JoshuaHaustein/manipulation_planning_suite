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
#include <mps/planner/util/Random.h>
#include <mps/planner/util/Time.h>
#include <mps/planner/pushing/PushPlannerDistanceMeasure.h>
// stl
#include <stack>
#include <mps/planner/ompl/control/NaiveControlSampler.h>
#include <mps/planner/pushing/oracle/OracleControlSampler.h>
#include <iomanip>

namespace mps {
    namespace planner {
        namespace pushing {
            namespace algorithm {
                struct PlanningStatistics {
                    unsigned int num_iterations;
                    unsigned int num_state_propagations;
                    unsigned int num_samples;
                    unsigned int num_nearest_neighbor_queries;
                    float runtime;
                    bool success;

                    void print(std::ostream& os) const{
                        os << "Planning statistics: \n";
                        os << "     num_iterations: " << num_iterations;
                        os << "     num_state_propagations: " << num_state_propagations;
                        os << "     num_samples: " << num_samples;
                        os << "     num_nearest_neighbor_queries: " << num_nearest_neighbor_queries;
                        os << "     run_time: " << std::fixed << runtime << std::setprecision(3);
                        os << "     success: " << success;
                        os << " \n";
                    }

                    void printCVS(std::ostream& os) const{
                        os << num_iterations << ", "
                           << num_state_propagations << ", "
                           << num_samples << ", "
                           << num_nearest_neighbor_queries << ", "
                           << runtime << ", "
                           << success << "\n";
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
                        std::shared_ptr<::ompl::base::GoalSampleableRegion> goal_region; // goal region
                        ::ompl::base::State* start_state; // start state of the problem (SimEnvWorldStateSpace)
                        std::string target_name;
                        std::string robot_name;
                        float time_out; // time out in seconds
                        float goal_bias;
                        float target_bias; // fraction of times the planner should focus at least on moving the target
                        float robot_bias; // fraction of times the planner should focus at least on moving the robot
                        unsigned int num_slice_neighbors; // number of nearest slice neighbors (only for SliceBasedOracleRRT)
                        float slice_volume;
                        std::vector<float> weights; // optional weights for the distance function
                        std::function<bool()> stopping_condition; // optionally a customized stopping condition
                        PlanningQuery(std::shared_ptr<::ompl::base::GoalSampleableRegion> goal_region,
                                      ::ompl::base::State* start_state,
                                      float time_out,
                                      const std::string& target_name,
                                      const std::string& robot_name);
                        PlanningQuery(const PlanningQuery& other);
                    };

                    class DebugDrawer {
                        // TODO this class may be overfit to a 2d planning case.
                    public:
                        DebugDrawer(sim_env::WorldViewerPtr world, unsigned int robot_id, unsigned int target_id);
                        ~DebugDrawer();
                        void addNewMotion(mps::planner::ompl::planning::essentials::MotionPtr motion);
                        void clear();
                        void drawStateTransition(const ompl::state::SimEnvObjectState* parent_state,
                                                 const ompl::state::SimEnvObjectState* new_state,
                                                 const Eigen::Vector4f& color);

                    private:
                        sim_env::WorldViewerPtr _world_viewer;
                        std::vector<sim_env::WorldViewer::Handle> _handles;
                        unsigned int _robot_id;
                        unsigned int _target_id;
                    };
                    typedef std::shared_ptr<DebugDrawer> DebugDrawerPtr;

                protected:
                    struct PlanningBlackboard {
                        PlanningQuery pq;
                        PlanningStatistics stats;
                        unsigned int target_id;
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
                    ::ompl::base::StateSamplerPtr _state_sampler;
                    mps::planner::ompl::state::SimEnvWorldStateSpacePtr _state_space;
                    mps::planner::pushing::PushPlannerDistanceMeasurePtr _distance_measure;
                    ::ompl::RNGPtr _rng;
                    DebugDrawerPtr _debug_drawer;

                private:
                    void setupBlackboard(PlanningBlackboard& pb);
                    mps::planner::util::time::Timer _timer;

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
                                          unsigned int k);
                    ~NaiveRearrangementRRT() override;
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

                class OracleRearrangementRRT : public RearrangementRRT {
                public:
                    OracleRearrangementRRT(::ompl::control::SpaceInformationPtr si,
                                          mps::planner::pushing::oracle::PushingOraclePtr pushing_oracle,
                                          mps::planner::pushing::oracle::RobotOraclePtr robot_oracle,
                                          const std::string& robot_name,
                                          const oracle::OracleControlSampler::Parameters& params=
                                            oracle::OracleControlSampler::Parameters());
                    ~OracleRearrangementRRT() override;
                    void setOracleSamplerParameters(const oracle::OracleControlSampler::Parameters& params);

                    bool extend(mps::planner::ompl::planning::essentials::MotionPtr start,
                                ::ompl::base::State* dest,
                                unsigned int active_obj_id,
                                mps::planner::ompl::planning::essentials::MotionPtr& last_motion,
                                PlanningBlackboard& pb) override;
                protected:
                    mps::planner::ompl::control::SimEnvStatePropagatorPtr _state_propagator;
                    mps::planner::pushing::oracle::OracleControlSampler _oracle_sampler;
                };

                class SliceBasedOracleRRT : public OracleRearrangementRRT {
                public:
                    SliceBasedOracleRRT(::ompl::control::SpaceInformationPtr si,
                                        mps::planner::pushing::oracle::PushingOraclePtr pushing_oracle,
                                        mps::planner::pushing::oracle::RobotOraclePtr robot_oracle,
                                        const std::string& robot_name,
                                        const oracle::OracleControlSampler::Parameters& params=
                                        oracle::OracleControlSampler::Parameters());
                    ~SliceBasedOracleRRT() override;
                    void setup(const PlanningQuery& pq, PlanningBlackboard& blackboard) override;
                    bool sample(mps::planner::ompl::planning::essentials::MotionPtr motion,
                                unsigned int& target_obj_id,
                                PlanningBlackboard& pb) override;
                    void selectTreeNode(const ompl::planning::essentials::MotionPtr& sample,
                                        ompl::planning::essentials::MotionPtr& selected_node,
                                        unsigned int& active_obj_id,
                                        bool sample_is_goal,
                                        PlanningBlackboard& pb) override;

                    void addToTree(mps::planner::ompl::planning::essentials::MotionPtr new_motion,
                                   mps::planner::ompl::planning::essentials::MotionPtr parent,
                                   PlanningBlackboard& pb) override;

                protected:
                    struct Slice {
                        typedef std::function<double(const mps::planner::ompl::planning::essentials::MotionPtr&,
                                                     const mps::planner::ompl::planning::essentials::MotionPtr&)> SliceDistanceFn;
                        Slice(mps::planner::ompl::planning::essentials::MotionPtr repr,
                              SliceDistanceFn distance_fn);
                        ~Slice();
                        void addSample(ompl::planning::essentials::MotionPtr motion);
                        std::shared_ptr<::ompl::NearestNeighbors<mps::planner::ompl::planning::essentials::MotionPtr> > slice_samples_nn;
                        std::vector<ompl::planning::essentials::MotionPtr> slice_samples_list;
                        mps::planner::ompl::planning::essentials::MotionPtr repr; // representative
                    };
                    typedef std::shared_ptr<Slice> SlicePtr;


                    struct WithinSliceDistance {
                        PushPlannerDistanceMeasure distance_measure;
                        double distance(const ompl::planning::essentials::MotionPtr& motion_a,
                                        const ompl::planning::essentials::MotionPtr& motion_b) const;
                        void setRobotId(unsigned int id);
                        explicit WithinSliceDistance(ompl::state::SimEnvWorldStateSpacePtr state_space,
                                            const std::vector<float>& weights=std::vector<float>());
                    };

                    struct SliceDistance {
                        PushPlannerDistanceMeasure distance_measure;
                        void setRobotId(unsigned int id);
                        double distance(const SlicePtr& slice_a, const SlicePtr& slice_b) const;
                        explicit SliceDistance(ompl::state::SimEnvWorldStateSpacePtr state_space,
                                               const std::vector<float>& weights=std::vector<float>());
                    };

                    SlicePtr getSlice(ompl::planning::essentials::MotionPtr motion) const;
                    float distanceToSlice(ompl::planning::essentials::MotionPtr motion, SlicePtr slice) const;
                    void getKSlices(ompl::planning::essentials::MotionPtr motion,
                                    unsigned int k,
                                    std::vector<SlicePtr>& slices) const;
                    float evaluateFeasibility(ompl::planning::essentials::MotionPtr from_motion,
                                              ompl::planning::essentials::MotionPtr to_motion,
                                              unsigned int robot_id,
                                              unsigned int target_id) const;

                    std::vector<SlicePtr> _slices_list;
                    std::shared_ptr<::ompl::NearestNeighbors<SlicePtr> > _slices_nn;
                    ::ompl::base::StateSamplerPtr _robot_state_sampler;
                    ::ompl::base::StateSpacePtr _robot_state_space;
                    WithinSliceDistance _within_slice_distance_fn;
                    SliceDistance _slice_distance_fn;
                    mps::planner::pushing::oracle::PushingOraclePtr _pushing_oracle;
                private:
                    SlicePtr _query_slice;
                };
            }
        }
    }
}
#endif //MANIPULATION_PLANNING_SUITE_RRT_H
