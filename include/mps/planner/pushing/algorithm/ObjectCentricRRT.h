//
// Created by joshua on 03/20/19.
//

#ifndef MANIPULATION_PLANNING_SUITE_OBJECT_CENTRIC_RRT_H
#define MANIPULATION_PLANNING_SUITE_OBJECT_CENTRIC_RRT_H

// MPS includes
#include <mps/planner/pushing/algorithm/RRT.h>

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
             * This algorithm combines several concepts, to efficiently compute rearrangement solutions:
             *  1. To sample pushing states and actions it uses a learned pushing policy and generator.
             *  2. To model the effects of pushing, it uses a physics model.
             *  3. To transition between different slices, it applies a greedy monotone backtracking based algorithm.
             */
            class ObjectCentricRRT : public OracleRearrangementRRT {
            public:
                ObjectCentricRRT(::ompl::control::SpaceInformationPtr si,
                    mps::planner::pushing::oracle::PushingOraclePtr pushing_oracle,
                    mps::planner::pushing::oracle::RobotOraclePtr robot_oracle,
                    const std::string& robot_name);
                ~ObjectCentricRRT() override;
                void setup(const PlanningQuery& pq, PlanningBlackboard& blackboard) override;
                bool extend(mps::planner::ompl::planning::essentials::MotionPtr start,
                    ::ompl::base::State* dest,
                    unsigned int active_obj_id,
                    mps::planner::ompl::planning::essentials::MotionPtr& last_motion,
                    PlanningBlackboard& pb);
                void selectTreeNode(const ompl::planning::essentials::MotionPtr& sample,
                    ompl::planning::essentials::MotionPtr& selected_node,
                    unsigned int& active_obj_id,
                    bool sample_is_goal,
                    PlanningBlackboard& pb) override;
                void addToTree(mps::planner::ompl::planning::essentials::MotionPtr new_motion,
                    mps::planner::ompl::planning::essentials::MotionPtr parent,
                    PlanningBlackboard& pb) override;

                // struct Slice {
                //     typedef std::function<double(const mps::planner::ompl::planning::essentials::MotionPtr&,
                //         const mps::planner::ompl::planning::essentials::MotionPtr&)>
                //         SliceDistanceFn;
                //     Slice(mps::planner::ompl::planning::essentials::MotionPtr repr,
                //         SliceDistanceFn distance_fn);
                //     ~Slice();
                //     void addSample(ompl::planning::essentials::MotionPtr motion);
                //     void clear();
                //     void reset(ompl::planning::essentials::MotionPtr repr);
                //     std::shared_ptr<::ompl::NearestNeighbors<mps::planner::ompl::planning::essentials::MotionPtr>> slice_samples_nn;
                //     std::vector<ompl::planning::essentials::MotionPtr> slice_samples_list;
                //     mps::planner::ompl::planning::essentials::MotionPtr repr; // representative
                // };
                // typedef std::shared_ptr<Slice> SlicePtr;
                // typedef std::shared_ptr<const Slice> SliceConstPtr;

            protected:
                // type definitions
                // struct WithinSliceDistance {
                //     ompl::state::SimEnvWorldStateDistanceMeasure distance_measure;
                //     double distance(const ompl::planning::essentials::MotionPtr& motion_a,
                //         const ompl::planning::essentials::MotionPtr& motion_b) const;
                //     void setRobotId(unsigned int id);
                //     explicit WithinSliceDistance(ompl::state::SimEnvWorldStateSpacePtr state_space,
                //         const std::vector<float>& weights = std::vector<float>());
                // };

                // struct SliceDistance {
                //     ompl::state::SimEnvWorldStateDistanceMeasure distance_measure;
                //     void setRobotId(unsigned int id);
                //     double distance(const SliceConstPtr& slice_a, const SliceConstPtr& slice_b) const;
                //     explicit SliceDistance(ompl::state::SimEnvWorldStateSpacePtr state_space,
                //         const std::vector<float>& weights = std::vector<float>());
                // };

                // typedef std::tuple<SlicePtr, float> ExtensionCandidateTuple;
                // // member functions
                // SlicePtr getSlice(ompl::planning::essentials::MotionPtr motion, PlanningBlackboard& pb) const;
                // float distanceToSlice(ompl::planning::essentials::MotionPtr motion, SlicePtr slice) const;
                // void projectSliceOnBall(SlicePtr sample_slice,
                //     SliceConstPtr center_slice,
                //     float radius,
                //     PlanningBlackboard& pb);

                // SlicePtr getNewSlice(ompl::planning::essentials::MotionPtr motion) const;
                // void cacheSlice(SlicePtr slice) const;
                // ExtensionCandidateTuple selectCandidateSlice(const std::vector<ExtensionCandidateTuple>& candidates) const;
                // member variables
                // std::vector<SlicePtr> _slices_list;
                // std::shared_ptr<::ompl::NearestNeighbors<SlicePtr>> _slices_nn;
                ::ompl::base::StateSamplerPtr _robot_state_sampler;
                ::ompl::base::StateSpacePtr _robot_state_space;
                // WithinSliceDistance _within_slice_distance_fn;
                // SliceDistance _slice_distance_fn;
                mps::planner::pushing::oracle::PushingOraclePtr _pushing_oracle;

            private:
                // mutable std::stack<SlicePtr> _slices_cache;
            };

            //////////////////////////////////////////////////////////////////////////////////////
            /////////////////////////////// Shortcutting algorithms //////////////////////////////
            //////////////////////////////////////////////////////////////////////////////////////
        }
    }
}
}
#endif //MANIPULATION_PLANNING_SUITE_RRT_H
