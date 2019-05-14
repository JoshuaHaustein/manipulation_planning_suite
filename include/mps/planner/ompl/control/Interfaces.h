//
// Created by joshua on 8/14/17.
//

#ifndef MANIPULATION_PLANNING_SUITE_VELOCITYCONTROL_H
#define MANIPULATION_PLANNING_SUITE_VELOCITYCONTROL_H

#include <Eigen/Core>
#include <mps/planner/util/Serialize.h>
#include <ompl/control/Control.h>
#include <sim_env/Controller.h>

namespace mps {
namespace planner {
    namespace ompl {
        namespace control {
            // Inteface for control spaces for which the control can be serialized (to save/load solutions).
            class SerializableControlSpace {
            public:
                virtual ~SerializableControlSpace() = 0;
                // Write information characterizing this control space to the given stream.
                virtual void serializeSpaceInformation(std::ostream& ostream) const = 0;
                // Read space information from given stream, return whether this control space is compatible.
                virtual bool deserializeSpaceInformation(std::istream& istream) = 0;
            };
            /**
                 * A RealValueParameterizedControl is a control that can be fully described
                 * by a finite number of real values (parameters).
                 */
            class RealValueParameterizedControl : public ::mps::planner::util::serialize::RealValueSerializable {
            public:
                virtual ~RealValueParameterizedControl() = 0;
                virtual Eigen::VectorXf getParameters() const = 0;
                virtual void getParameters(Eigen::VectorXf& params) const = 0;
                virtual void setParameters(const Eigen::VectorXf& params) = 0;
                virtual unsigned int getNumParameters() const = 0;

                // for oracle serialization
                void serializeInNumbers(std::ostream& ostream) const override;
                void deserializeFromNumbers(std::istream& istream) override;
                unsigned int getNumNumbers() const override;
            };

            // A timed control is a function T -> U where T is time, and U some target control space (Positions or Velocities).
            // T is an interval [0, d] where d can be obtained calling getDuration(). getTarget(t, target)
            // allows to retrieve the target value at time t. In addition, a TimedControl may provide a PositionConstraint
            // or VelocityConstraint to the controller. These constraints are implemented through projection functions.
            // The default implementation of the respective getters return empty projection functions, meaning there are no
            // constraints for the controller and the controller may do anything to follow its target.
            class TimedControl {
            public:
                virtual ~TimedControl() = 0;
                virtual float getDuration() const = 0;
                virtual void getTarget(float t, Eigen::VectorXf& target) const = 0;
                virtual sim_env::RobotController::PositionProjectionFn getPositionConstraintProjection() const;
                virtual sim_env::RobotController::VelocityProjectionFn getVelocityConstraintProjection() const;
            };

            /**
             *  A position control is a function [0, T] -> C that maps time to desired robot state.
             */
            class PositionControl : public TimedControl,
                                    public ::ompl::control::Control {
            public:
                ~PositionControl() override = 0;
                virtual Eigen::VectorXf getPosition(float t) const = 0;
                virtual void getPosition(float t, Eigen::VectorXf& pos) const = 0;
                void getTarget(float t, Eigen::VectorXf& target) const override;
            };

            /**
              * A semi-dynamic control is a control with finite duration T after which the robot is
              * expected to be at rest.
              * For this, the total duration of this control is segmented into two segments
              * T = T_pre_rest + T_rest, where t in [0, T_pre_rest) is the duration in which the robot may move,
              * whereas for t in [T_active, T_rest] the robot should rest.
              */
            class SemiDynamicControl {
            public:
                virtual ~SemiDynamicControl() = 0;
                virtual float getPreRestDuration() const = 0;
                virtual float getRestDuration() const = 0;
                virtual void addRestDuration(float dt) = 0;
                virtual void setRestDuration(float dt) = 0;
            };

            /**
                 * A VelocityControl is a velocity profile [0, T] -> V, where V is the velocity space of a robot,
                 * T is the maximum duration of the velocity profile (which may be infinite). For each time t in [0, T]
                 * ([0, infinity)) the velocity profile defines a desired velocity v(t) in V.
                 */
            class VelocityControl : public TimedControl, public ::ompl::control::Control {
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

                void getTarget(float t, Eigen::VectorXf& target) const override;
            };

            /**
                 * A semi-dynamic velocity control is a velocity control for which T is finite and v(T) = 0.
                 * Furthermore, the total duration of this control can be segmented into two segments
                 * T = T_active + T_rest, where t in [0, T_active) is the duration in which v(t) may be non-zero,
                 * whereas for t in [T_active, T_rest] it is v(t) = 0.
                 */
            // class SemiDynamicVelocityControl : public virtual VelocityControl {
            // public:
            //     // TODO what about copy constructor etc
            //     ~SemiDynamicVelocityControl() override = 0;
            //     /**
            //          * Return the resting time of this control
            //          * @return resting time in seconds
            //          */
            //     virtual float getRestTime() const = 0;
            //     /**
            //          * Add the given rest time to the resting time of this control.
            //          * @param dt  - resting time in seconds
            //          */
            //     virtual void addRestTime(float dt) = 0;
            //     /**
            //          * Set the given rest time.
            //          * @param t  - rest time in seconds
            //          */
            //     virtual void setRestTime(float t) = 0;
            // };
        }
    }
}
}

#endif //MANIPULATION_PLANNING_SUITE_VELOCITYCONTROL_H
