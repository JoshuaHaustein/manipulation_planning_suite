//
// Created by joshua on 8/29/17.
//

#ifndef MANIPULATION_PLANNING_SUITE_RELOCATEGOAL_H
#define MANIPULATION_PLANNING_SUITE_RELOCATEGOAL_H

#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/SpaceInformation.h>
#include <iostream>
#include <Eigen/Dense>
#include <mps/planner/util/Random.h>

namespace mps {
    namespace planner {
        namespace ompl {
            namespace state {
                namespace goal {
                    struct RelocationGoalSpecification {
                        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
                        std::string object_name;
                        Eigen::Vector3f goal_position;
                        Eigen::Quaternionf goal_orientation;
                        float position_tolerance;
                        float orientation_tolerance;
                        unsigned int id;
                        /**
                         * Default constructor. WARNING: Sets default values that are not a valid goal!
                         */
                        RelocationGoalSpecification();
                        /**
                         * A RelocationGoalSpecification specifies where a certain object is supposed to be relocated to.
                         * @param name - the target object to relocate
                         * @param position - the desired goal position of the target object
                         * @param orienation - the desired goal orientation of the target object (TODO not supported yet)
                         * @param position_tolerance - allowed error in position
                         * @param angle_tolerance - allowed error in angle
                        */
                        RelocationGoalSpecification(const std::string& name,
                                                    const Eigen::Vector3f& position,
                                                    const Eigen::Quaternionf& orientation,
                                                    float position_tolerance,
                                                    float angle_tolerance);
                        RelocationGoalSpecification(const RelocationGoalSpecification& other);
                        RelocationGoalSpecification& operator=(const RelocationGoalSpecification& other);
                    };

                    /**
                     * An ObjectsRelocationGoal represents the goal of relocating
                     * one or multiple objects in a scene. The goal is fulfilled if all target objects,
                     * are within proximity to a target pose. The target objects as well as
                     * their target poses can be specified by the user.
                     * TODO: This class currently only supports position goals in SE(2), ignoring orientation
                     */
                    class ObjectsRelocationGoal : public ::ompl::base::GoalSampleableRegion
                    {
                      public:
                        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
                        /**
                         * Creates a new ObjectsRelocationGoal where only a single object is required to be relocated.
                         * @param si - space information
                         * @param goal_specification - a single relocation goal
                         * @param position_tolerance - a position tolerance, i.e. a radius around the target position, which is acceptable
                         * @param orientation_tolerance - orientation tolerance, i.e. maximal error in orientation that is acceptable (TODO not supported yet)
                         */
                        ObjectsRelocationGoal(::ompl::base::SpaceInformationPtr si,
                                             const RelocationGoalSpecification& goal_specification);
                        /**
                         * Creates a new ObjectsRelocationGoal where multiple objects are required to be relocated.
                         * @param si - space information
                         * @param goal_specifications - a vector of relocation goals - there must be at most one per object
                         */
                        ObjectsRelocationGoal(::ompl::base::SpaceInformationPtr si,
                                             const std::vector<RelocationGoalSpecification>& goal_specifications);
                        ~ObjectsRelocationGoal();

                        /**
                         * Samples a new goal state.
                         * @param state - an allocated SimEnvWorldState
                         */
                        void sampleGoal(::ompl::base::State* state) const override;
                        unsigned int maxSampleCount() const override;
                        double distanceGoal(const ::ompl::base::State *st) const override;
                        void print(std::ostream& out = std::cout) const override;

                        /**
                         * Randomly selects an object from the target objects and returns its index.
                         */
                        unsigned int sampleTargetObjectIndex() const;
                    private:
                        std::vector<RelocationGoalSpecification> _goal_specifications;
                        std::vector<unsigned int> _target_indices;
                        ::ompl::base::StateSamplerPtr _state_sampler;
                        ::ompl::RNGPtr _rng;
                        ::ompl::base::StateValidityCheckerPtr _validity_checker;
                        unsigned int _max_num_attempts;

                    };
                    typedef std::shared_ptr<ObjectsRelocationGoal> ObjectsRelocationGoalPtr;
                }
            }
        }
    }
}
#endif //MANIPULATION_PLANNING_SUITE_RELOCATEGOAL_H
