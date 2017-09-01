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
#include <mps/planner/util/Random.h>
#include <mps/planner/pushing/PushPlannerDistanceMeasure.h>
// stl
#include <stack>

namespace mps {
    namespace planner {
        namespace pushing {
            namespace algorithm {
                // TODO we could inherit from ::ompl::base::Planner or could define our own planner interface, if needed
                /**
                 * This class implements the semi-dynamic RRT algorithm. The semi-dynamic RRT algorithm
                 * is essentially the same as the normal RRT algorithm with the slight difference, that it utilizes
                 * a state propagator that may adjust the control. The term 'semi-dynamic' refers to such an adjustment
                 * in the case of push planning. Here, the state propagator may add an additional waiting time to a robot
                 * control action to allow the environment to come to rest.
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
                        float time_out; // time out in seconds
                        float goal_bias;
                        std::function<bool()> stopping_condition; // optionally a customized stopping condition
                        PlanningQuery(std::shared_ptr<::ompl::base::GoalSampleableRegion> goal_region,
                                      ::ompl::base::State* start_state,
                                      float time_out);
                    };

                    class Motion;
                    typedef std::shared_ptr<Motion> MotionPtr;
                    typedef std::shared_ptr<const Motion> MotionConstPtr;

                    class Motion {
                    public:
                        Motion() = delete;
                        Motion(::ompl::control::SpaceInformationPtr si);
                        Motion(const Motion& other);
                        ~Motion();
                        Motion& operator=(const Motion& other);
                        ::ompl::base::State* getState();
                        ::ompl::control::Control* getControl();
                        MotionPtr getParent();
                        MotionConstPtr getConstParent();
                        void setParent(MotionPtr parent);
                    private:
                        std::weak_ptr<::ompl::control::SpaceInformation> _weak_si;
                        ::ompl::base::State* _state;
                        ::ompl::control::Control* _control;
                        MotionPtr _parent;
                    };


                    class Path : public ::ompl::base::Path {
                    public:
                        Path(::ompl::control::SpaceInformationPtr si);
                        ~Path();

                        double length() const override;
                        ::ompl::base::Cost cost(const ::ompl::base::OptimizationObjectivePtr& oo) const override;
                        bool check() const override;
                        void print(std::ostream& out) const override;

                        /**
                         * Append a motion to this path. The motion is not copied!
                         * @param motion - motion to append to this path.
                         */
                        void append(MotionPtr motion);
                        /**
                         * Resets this path and initializes this path by backtracking the path leading
                         * to motion. None of the motions are copied!
                         * @param motion - final motion of a path.
                         */
                        void initBacktrackMotion(MotionPtr motion);
                        /**
                         * Clear this path.
                         */
                        void clear();
                        unsigned int getNumMotions() const;
                        MotionPtr getMotion(unsigned int i);
                        MotionConstPtr getConstMotion(unsigned int i) const;
                        //TODO could also define iterator for this

                    private:
                        ::ompl::control::SpaceInformationPtr _sic;
                        std::vector<MotionPtr> _motions;
                        double _length;
                    };

                    /**
                     * Creates a new semi-dynamic RRT algorithm. The state propagator provided in si is expected to be
                     * of type SimEnvStatePropagator. Accordingly the control space is expected to be a space of controls
                     * that inherit from SemiDynamicVelocityControl.
                     * @param si
                     */
                    SemiDynamicRRT(::ompl::control::SpaceInformationPtr si);
                    ~SemiDynamicRRT();

                    void setup();

                    bool plan(const PlanningQuery& pq, Path& path);

                private:
                    ::ompl::control::SpaceInformationPtr _si;
                    ::ompl::base::StateSamplerPtr _state_sampler;
                    ::ompl::control::DirectedControlSamplerPtr _control_sampler;
                    mps::planner::ompl::control::SimEnvStatePropagatorPtr _state_propagator;
                    mps::planner::pushing::PushPlannerDistanceMeasurePtr _distance_measure;
                    ::ompl::RNGPtr _rng;
                    unsigned int _num_objects;

                    std::string _log_prefix;
                    bool _is_setup;
                    std::shared_ptr<::ompl::NearestNeighbors< MotionPtr > > _tree;
                    std::stack<MotionPtr> _motions_cache;

                    MotionPtr getNewMotion();
                    void cacheMotion(MotionPtr ptr);
                };
            }
        }
    }
}
#endif //MANIPULATION_PLANNING_SUITE_RRT_H
