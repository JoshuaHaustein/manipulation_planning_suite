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
#include <mps/planner/ompl/control/NaiveControlSampler.h>
#include <mps/planner/ompl/control/SimEnvStatePropagator.h>
#include <mps/planner/pushing/algorithm/RearrangementPlanner.h>
#include <mps/planner/pushing/oracle/OracleControlSampler.h>
#include <mps/planner/util/Random.h>
#include <mps/sdf/SDF.h>

namespace mps {
namespace planner {
    namespace pushing {
        namespace algorithm {
            /**
                 * This class implements a semi-dynamic RRT algorithm for non-prehensile rearrangement.
                 * The algorithm searches for a sequence of robot actions that push target objects into some goal region.
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
                 * 
                 * While a SingleExtendRRT algorithm solves rearrangement problems with multiple objects, its
                 * extend function only focusses at transporting one object at a time (multiple objects may be transported
                 * coincidentally).
                 */
            class SingleExtendRRT : public RearrangementPlanner {
            public:
                /**
                     * Creates a new semi-dynamic RRT algorithm. The state propagator provided in si is expected to be
                     * of type SimEnvStatePropagator. Accordingly the control space is expected to be a space of controls
                     * that inherit from SemiDynamicVelocityControl.
                     * @param si
                     */
                SingleExtendRRT(::ompl::control::SpaceInformationPtr si);
                virtual ~SingleExtendRRT() = 0;
                bool plan(PlanningQueryPtr pq, PlanningStatistics& stats) override;
                PlanningQueryPtr createPlanningQuery(mps::planner::ompl::state::goal::ObjectsRelocationGoalPtr goal_region,
                    mps::planner::ompl::state::SimEnvWorldState* start_state, const std::string& robot_name, float timeout) override;
                bool isGoalPath(mps::planner::ompl::planning::essentials::PathConstPtr path,
                    const mps::planner::ompl::state::SimEnvWorldState* start,
                    RearrangementPlanner::PlanningQueryPtr pq,
                    mps::planner::ompl::planning::essentials::PathPtr updated_path)
                    override;

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
                    PlanningBlackboard& pb);
                virtual double treeDistanceFunction(const mps::planner::ompl::planning::essentials::MotionPtr& a,
                    const mps::planner::ompl::planning::essentials::MotionPtr& b) const;

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

                ::ompl::base::ValidStateSamplerPtr _state_sampler;
                mps::planner::ompl::control::SimEnvStatePropagatorPtr _state_propagator;
                mps::planner::ompl::state::SimEnvWorldStateDistanceMeasurePtr _distance_measure;
                ::ompl::RNGPtr _rng;
                mutable MotionCache<> _motion_cache;
                // Parameters
                float _goal_bias; // fraction of times the planner should attempt to connect to a goal configuration
                float _target_bias; // fraction of times the planner should focus at least on moving the target objects
                float _robot_bias; // fraction of times the planner should focus at least on moving the robot

            private:
                std::string _log_prefix;
                std::shared_ptr<::ompl::NearestNeighbors<mps::planner::ompl::planning::essentials::MotionPtr>> _tree;
            };

            typedef std::shared_ptr<SingleExtendRRT> SingleExtendRRTPtr;
            typedef std::shared_ptr<const SingleExtendRRT> SingleExtendRRTConstPtr;
            typedef std::weak_ptr<SingleExtendRRT> SingleExtendRRTWeakPtr;
            typedef std::weak_ptr<const SingleExtendRRT> SingleExtendRRTWeakConstPtr;

            class NaiveRearrangementRRT : public SingleExtendRRT {
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

            class HybridActionRRT : public SingleExtendRRT {
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
                void sampleActionSequence(std::vector<::ompl::control::Control*>& controls,
                    mps::planner::ompl::planning::essentials::MotionPtr start,
                    ::ompl::base::State* dest,
                    PlanningBlackboard& pb);
                void forwardPropagateActionSequence(const std::vector<::ompl::control::Control*>& controls,
                    mps::planner::ompl::planning::essentials::MotionPtr start,
                    std::vector<mps::planner::ompl::planning::essentials::MotionPtr>& state_action_seq,
                    PlanningBlackboard& pb);
                void freeMotionList(std::vector<mps::planner::ompl::planning::essentials::MotionPtr>& motions);
                mps::planner::pushing::oracle::OracleControlSamplerPtr _oracle_sampler;
            };
            typedef std::shared_ptr<HybridActionRRT> HybridActionRRTPtr;
            typedef std::shared_ptr<const HybridActionRRT> HybridActionRRTConstPtr;
            typedef std::weak_ptr<HybridActionRRT> HybridActionRRTWeakPtr;
            typedef std::weak_ptr<const HybridActionRRT> HybridActionRRTWeakConstPtr;

            class OracleRearrangementRRT : public SingleExtendRRT {
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
                void extendStep(const std::vector<::ompl::control::Control*>& controls,
                    const mps::planner::ompl::planning::essentials::MotionPtr& start_motion,
                    mps::planner::ompl::planning::essentials::MotionPtr& result_motion,
                    PlanningBlackboard& pb,
                    bool& extension_success,
                    bool& goal_reached);

                float _action_randomness; // parameter in [0, 1] that determines randomness of action sampling (prand)
                float _feasible_state_noise; // probability (in [0,1]) to sample a feasible state uniformly rather than from the oracle
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
                    PlanningBlackboard& pb) override;
                void addToTree(mps::planner::ompl::planning::essentials::MotionPtr new_motion,
                    mps::planner::ompl::planning::essentials::MotionPtr parent,
                    PlanningBlackboard& pb) override;

            protected:
                // member functions
                SlicePtr getSlice(ompl::planning::essentials::MotionPtr motion, PlanningBlackboard& pb) const;
                float distanceToSlice(ompl::planning::essentials::MotionPtr motion, SlicePtr slice) const;
                // member variables
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
        }
    }
}
}
#endif //MANIPULATION_PLANNING_SUITE_RRT_H
