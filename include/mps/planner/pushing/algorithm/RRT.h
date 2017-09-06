//
// Created by joshua on 8/14/17.
//

#ifndef MANIPULATION_PLANNING_SUITE_RRT_H
#define MANIPULATION_PLANNING_SUITE_RRT_H

// OMPL includes
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
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

namespace mps {
    namespace planner {
        namespace pushing {
            namespace algorithm {
                // TODO we could inherit from ::ompl::base::Planner or could define our own planner interface, if needed
                /**
                 * This class implements the semi-dynamic RRT algorithm for non-prehensile rearrangement.
                 * The algorithm searches for a sequence of robot actions that push a target object into some goal region.
                 * The special aspect about the semi-dynamic RRT is that the state propagator may add an
                 * additional waiting time to a robot control action to allow the environment to come to rest. After
                 * each action. This allows utilizing dynamical physical interactions between the robot and its environment
                 * without planning on full state space.
                 *
                 * This planner operates on SimEnvWorldStateSpace only as it utilizes the SimEnvStatePropagator.
                 * Also, the control space needs to be compatible to SemiDynamicVelocityControl, i.e. the controls
                 * need to be descendents of SemiDynamicVelocityControl.
                 */
                class SemiDynamicRRT {
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
                        std::vector<float> weights; // optional weights for the distance function
                        std::function<bool()> stopping_condition; // optionally a customized stopping condition
                        PlanningQuery(std::shared_ptr<::ompl::base::GoalSampleableRegion> goal_region,
                                      ::ompl::base::State* start_state,
                                      float time_out,
                                      const std::string& target_name,
                                      const std::string& robot_name);
                    };


                    // TODO this class may be overfit to a 2d planning case.
                    class DebugDrawer {
                    public:
                        DebugDrawer(sim_env::WorldViewerPtr world);
                        ~DebugDrawer();
                        void addNewMotion(mps::planner::ompl::planning::essentials::MotionPtr motion);
                        void clear();
                        void drawStateTransition(const ompl::state::SimEnvObjectState* parent_state,
                                                 const ompl::state::SimEnvObjectState* new_state,
                                                 const Eigen::Vector4f& color);

                    private:
                        sim_env::WorldViewerPtr _world_viewer;
                        std::vector<sim_env::WorldViewer::Handle> _handles;
                    };
                    typedef std::shared_ptr<DebugDrawer> DebugDrawerPtr;


                    /**
                     * Creates a new semi-dynamic RRT algorithm. The state propagator provided in si is expected to be
                     * of type SimEnvStatePropagator. Accordingly the control space is expected to be a space of controls
                     * that inherit from SemiDynamicVelocityControl.
                     * @param si
                     */
                    SemiDynamicRRT(::ompl::control::SpaceInformationPtr si);
                    ~SemiDynamicRRT();

                    void setup();

                    bool plan(const PlanningQuery& pq, mps::planner::ompl::planning::essentials::PathPtr path);

                    void setDebugDrawer(DebugDrawerPtr debug_drawer);

                private:
                    ::ompl::control::SpaceInformationPtr _si;
                    mps::planner::ompl::state::SimEnvWorldStateSpacePtr _state_space;
                    ::ompl::base::StateSamplerPtr _state_sampler;
                    ::ompl::control::DirectedControlSamplerPtr _control_sampler;
                    mps::planner::ompl::control::SimEnvStatePropagatorPtr _state_propagator;
                    mps::planner::pushing::PushPlannerDistanceMeasurePtr _distance_measure;
                    ::ompl::RNGPtr _rng;
                    unsigned int _num_objects;
                    mps::planner::util::time::Timer _timer;

                    std::string _log_prefix;
                    bool _is_setup;
                    std::shared_ptr<::ompl::NearestNeighbors< mps::planner::ompl::planning::essentials::MotionPtr > > _tree;
                    std::stack<mps::planner::ompl::planning::essentials::MotionPtr> _motions_cache;

                    DebugDrawerPtr _debug_drawer;

                    unsigned int sampleActiveObject(const PlanningQuery& pq,
                                                    unsigned int target_id,
                                                    unsigned int robot_id) const;
                    mps::planner::ompl::planning::essentials::MotionPtr getNewMotion();
                    void cacheMotion(mps::planner::ompl::planning::essentials::MotionPtr ptr);
                    double treeDistanceFunction(const mps::planner::ompl::planning::essentials::MotionPtr& a,
                                                const mps::planner::ompl::planning::essentials::MotionPtr& b) const;
                };
                typedef std::shared_ptr<SemiDynamicRRT> SemiDynamicRRTPtr;
                typedef std::shared_ptr<const SemiDynamicRRT> SemiDynamicRRTConstPtr;
                typedef std::weak_ptr<SemiDynamicRRT> SemiDynamicRRTWeakPtr;
                typedef std::weak_ptr<const SemiDynamicRRT> SemiDynamicRRTWeakConstPtr;
            }
        }
    }
}
#endif //MANIPULATION_PLANNING_SUITE_RRT_H
