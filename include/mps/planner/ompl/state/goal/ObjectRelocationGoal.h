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
                    /**
                     * An ObjectRelocationGoal represents the goal of relocating
                     * an object in a scene. The goal is fulfilled if a particular object, the target object,
                     * is within proximity to a target pose. Both the target object and
                     * the the target pose can be specified by the user.
                     * TODO: This class currently only supports position goals in SE(2), ignoring orientation
                     */
                    class ObjectRelocationGoal : public ::ompl::base::GoalSampleableRegion {
                    public:
                        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
                        /**
                         * Creates a new ObjectRelocationGoal.
                         * @param si - space information
                         * @param target_obj  - the target object to relocate
                         * @param goal_position  - the desired goal position of the target object
                         * @param goal_orientation - the desired goal orientation of the target object (TODO not supported yet)
                         * @param position_tolerance - a position tolerance, i.e. a radius around the target position, which is acceptable
                         * @param orientation_tolerance - orientation tolerance, i.e. maximal error in orientation that is acceptable (TODO not supported yet)
                         */
                        ObjectRelocationGoal(::ompl::base::SpaceInformationPtr si,
                                             const std::string& target_obj,
                                             const Eigen::Vector3f& goal_position,
                                             const Eigen::Quaternionf& goal_orientation,
                                             float position_tolerance,
                                             float orientation_tolerance);
                        ~ObjectRelocationGoal();

                        /**
                         * Samples a new goal state.
                         * @param state - an allocated SimEnvWorldState
                         */
                        void sampleGoal(::ompl::base::State* state) const override;
                        unsigned int maxSampleCount() const override;
                        double distanceGoal(const ::ompl::base::State *st) const override;
                        void print(std::ostream& out = std::cout) const override;
                    private:
                        std::string _target_name;
                        const Eigen::Vector3f _goal_position;
                        const Eigen::Quaternionf _goal_orientation;
                        const float _position_tolerance;
                        const float _orientation_tolerance;
                        unsigned int _target_id;
                        ::ompl::base::StateSamplerPtr _state_sampler;
                        ::ompl::RNGPtr _rng;

                    };
                }
            }
        }
    }
}
#endif //MANIPULATION_PLANNING_SUITE_RELOCATEGOAL_H
