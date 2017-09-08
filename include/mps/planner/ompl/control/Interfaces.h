//
// Created by joshua on 8/14/17.
//

#ifndef MANIPULATION_PLANNING_SUITE_VELOCITYCONTROL_H
#define MANIPULATION_PLANNING_SUITE_VELOCITYCONTROL_H

#include <ompl/control/Control.h>
#include <mps/planner/util/Serialize.h>
#include <Eigen/Core>

namespace mps {
    namespace planner {
        namespace ompl {
            namespace control {
                /**
                 * A RealValueParameterizedControl is a control that can be fully described
                 * by a finite number of real values (parameters).
                 */
                class RealValueParameterizedControl :
                        public ::ompl::control::Control,
                        public ::mps::planner::util::serialize::RealValueSerializable
                {
                public:
                    ~RealValueParameterizedControl() override = 0;
                    virtual Eigen::VectorXf getParameters() const = 0;
                    virtual void getParameters(Eigen::VectorXf& params) const = 0;
                    virtual void setParameters(const Eigen::VectorXf& params) = 0;
                    // for oracle serialization
                    void serializeInNumbers(std::ostream& ostream) const override;
                };

                /**
                 * A VelocityControl is a velocity profile [0, T] -> V, where V is the velocity space of a robot,
                 * T is the maximum duration of the velocity profile (which may be infinite). For each time t in [0, T]
                 * ([0, infinity)) the velocity profile defines a desired velocity v(t) in V.
                 */
                class VelocityControl : public RealValueParameterizedControl {
                public:
                    // TODO what about copy constructor etc
                    ~VelocityControl() override = 0;
                    /**
                     * Returns the desired velocity for each degree of freedom at the given time
                     * (relative to the start of this control).
                     * @param dt - time relative to the start of this control (in seconds)
                     * @return - Eigen::VectorXf containing a velocity for each dof (usually in m/s, rad/s)
                     */
                    virtual Eigen::VectorXf getVelocity(float dt) const = 0;
                    /**
                     * Stores the desired velocity for each degree of freedom at the given time in vel.
                     * @param dt - time relative to the start of this control (in seconds)
                     * @param vel - output vector in which the velocity is written. The vector is scaled to the proper dimension
                     */
                    virtual void getVelocity(float dt, Eigen::VectorXf& vel) const = 0;
                    /**
                     * Returns the maximal duration of this action in seconds.
                     * @return [0, infinity), where infinity = std::numeric_limits<float>::infinity()
                     */
                    virtual float getMaxDuration() const = 0;

                };

                /**
                 * A semi-dynamic velocity control is a velocity control for which T is finite and v(T) = 0.
                 * Furthermore, the total duration of this control can be segmented into two segments
                 * T = T_active + T_rest, where t in [0, T_active) is the duration in which v(t) may be non-zero,
                 * whereas for t in [T_active, T_rest] it is v(t) = 0.
                 */
                class SemiDynamicVelocityControl : public virtual VelocityControl {
                public:
                    // TODO what about copy constructor etc
                    ~SemiDynamicVelocityControl() override = 0;
                    /**
                     * Return the resting time of this control
                     * @return resting time in seconds
                     */
                    virtual float getRestTime() const = 0;
                    /**
                     * Add the given rest time to the resting time of this control.
                     * @param dt  - resting time in seconds
                     */
                    virtual void addRestTime(float dt) = 0;
                    /**
                     * Set the given rest time.
                     * @param t  - rest time in seconds
                     */
                    virtual void setRestTime(float t) = 0;
                };
            }
        }
    }
}

#endif //MANIPULATION_PLANNING_SUITE_VELOCITYCONTROL_H
