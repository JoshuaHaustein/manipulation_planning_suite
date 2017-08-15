//
// Created by joshua on 8/14/17.
//

#ifndef MANIPULATION_PLANNING_SUITE_RAMPVELOCITYCONTROL_H
#define MANIPULATION_PLANNING_SUITE_RAMPVELOCITYCONTROL_H

#include <manipulation_planning_suite/planner/ompl/control/Interfaces.h>
#include <ompl/control/ControlSpace.h>

namespace mps {
    namespace planner {
        namespace ompl {
            namespace control {
                /**
                 * The ramp velocity control is a simple velocity control that follows a ramp-shaped velocity profile.
                 * All degrees of freedom move synchronously.
                 */
                class RampVelocityControl : public virtual FiniteRestingVelocityControl {
                public:
                    RampVelocityControl(const Eigen::VectorXf& max_accelerations);
                    ~RampVelocityControl();
                    /**
                     * Set the maximal velocities, i.e. the peak velocity of the ramp for each dof, and the duration
                     * of maintaining the peak.
                     * @param max_velocities
                     * @param duration
                     */
                    void setMaxVelocities(const Eigen::VectorXf& max_velocities, float duration);

                    /**
                     * Set the parameters for this control. The first n-1 entries of parameters are the peak velocity,
                     * the last entry is the peak duration.
                     * @param parameters
                     */
                    void setParameters(const Eigen::VectorXf& parameters);
                    virtual Eigen::VectorXf getParameters() const;
                    virtual void getParameters(Eigen::VectorXf& params) const;

                    /**
                     * Returns the maximal velocities set for this action.
                     * @return
                     */
                    Eigen::VectorXf getMaxVelocities() const;
                    void getMaxVelocities(Eigen::VectorXf& vel) const;
                    virtual Eigen::VectorXf getVelocity(float dt) const;
                private:
                    Eigen::VectorXf _max_accelerations;
                    Eigen::VectorXf _max_velocities;
                    float acceleration_time;
                };

                class RampVelocityControlSpace : public ::ompl::control::ControlSpace {
                public:
                    /**
                     * Creates a new ramp velocity control space.
                     * @param velocity_limits - absolute maximal velocity for each dof
                     * @param acceleration_limits - absolute maximal acceleration for each dof
                     * @param duration_limits - duration limits (min, max)
                     */
                    RampVelocityControlSpace(const Eigen::VectorXf& velocity_limits,
                                             const Eigen::VectorXf& acceleration_limits,
                                             const Eigen::Array2f& duration_limits);

                    //TODO implement rest

                };
            }
        }
    }
}

#endif //MANIPULATION_PLANNING_SUITE_RAMPVELOCITYCONTROL_H
