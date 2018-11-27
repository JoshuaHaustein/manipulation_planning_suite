//
// Created by joshua on 8/14/17.
//

#ifndef MANIPULATION_PLANNING_SUITE_RAMPVELOCITYCONTROL_H
#define MANIPULATION_PLANNING_SUITE_RAMPVELOCITYCONTROL_H

#include <memory>
#include <mps/planner/ompl/control/Interfaces.h>
#include <ompl/control/ControlSpace.h>

namespace mps {
namespace planner {
    namespace ompl {
        namespace control {

            class RampVelocityControlSpace;
            typedef std::shared_ptr<RampVelocityControlSpace> RampVelocityControlSpacePtr;
            typedef std::weak_ptr<RampVelocityControlSpace> RampVelocityControlSpaceWeakPtr;
            typedef std::shared_ptr<const RampVelocityControlSpace> RampVelocityControlSpaceConstPtr;
            typedef std::weak_ptr<const RampVelocityControlSpace> RampVelocityControlSpaceConstWeakPtr;

            class RampVelocityControl;
            typedef std::shared_ptr<RampVelocityControl> RampVelocityControlPtr;
            typedef std::weak_ptr<RampVelocityControl> RampVelocityControlWeakPtr;
            typedef std::shared_ptr<const RampVelocityControl> RampVelocityControlConstPtr;
            typedef std::weak_ptr<const RampVelocityControl> RampVelocityControlConstWeakPtr;

            /**
                 * The ramp velocity control is a simple velocity control that follows a ramp-shaped velocity profile.
                 * All degrees of freedom move synchronously.
                 */
            class RampVelocityControl : public virtual SemiDynamicVelocityControl {
            public:
                RampVelocityControl(RampVelocityControlSpaceConstPtr control_space);
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
                void setParameters(const Eigen::VectorXf& parameters) override;
                Eigen::VectorXf getParameters() const override;
                void getParameters(Eigen::VectorXf& params) const override;
                unsigned int getNumParameters() const override;

                /**
                     * Initialize from a local twist (for holonomic SE(2) robots). 
                     * A local twist is a tuple (v, w, theta, t), where
                     *  v in (0, v_max) is the translational velocity
                     *  w in (-w_max, w_max) the angular velocity
                     *  theta in [0, 2pi) the direction of the translational velocity relative to the robot
                     *  t in [0, t_max] the duration of the plateau
                     * @param robot_pose - starting pose of the robot
                     * @param ltwist - the local twist
                     * TODO this is specific to SE(2) robots
                     */
                void setFromLocalTwist(const Eigen::Vector3f& robot_pose, const Eigen::Vector4f& ltwist);

                /**
                     * Returns the maximal velocities set for this action.
                     * @return
                     */
                Eigen::VectorXf getMaxVelocities() const;
                void getMaxVelocities(Eigen::VectorXf& vel) const;

                /** VelocityProfile interface **/
                Eigen::VectorXf getVelocity(float dt) const override;
                void getVelocity(float dt, Eigen::VectorXf& vel) const override;
                float getMaxDuration() const override;

                float getAccelerationTime() const;

                /** FiniteRestingVelocityControl interface **/
                float getRestTime() const override;
                void addRestTime(float dt) override;
                void setRestTime(float t) override;

            private:
                RampVelocityControlSpaceConstWeakPtr _control_space;
                Eigen::VectorXf _accelerations;
                Eigen::VectorXf _max_velocities;
                float _plateau_duration;
                float _acceleration_duration;
                float _rest_time;
                unsigned int _num_dofs;

                void computeRamp();
            };

            class RampVelocityControlSampler : public ::ompl::control::ControlSampler {
            public:
                explicit RampVelocityControlSampler(RampVelocityControlSpaceConstPtr control_space);
                ~RampVelocityControlSampler() override;

                void sample(::ompl::control::Control* control) override;

            private:
                const RampVelocityControlSpaceConstWeakPtr _control_space;
                double* _values_buffer;
            };

            class RampVelocityControlSpace : public ::ompl::control::ControlSpace,
                                             public std::enable_shared_from_this<RampVelocityControlSpace> {
                friend class RampVelocityControlSampler;

            public:
                struct ControlLimits {
                    Eigen::VectorXf velocity_limits;
                    Eigen::VectorXf acceleration_limits;
                    Eigen::Array2f duration_limits;
                    ControlLimits(const Eigen::VectorXf& velocity_limits,
                        const Eigen::VectorXf& acceleration_limits,
                        const Eigen::Array2f& duration_limits);
                    ControlLimits(); // without initialization
                };
                /**
                     * Defines a control subspace within the control space.
                     * The components within this subspace are sampled together from a ball with
                     * radius limited by norm_limits[1]
                     */
                struct ControlSubspace {
                    ControlSubspace(const Eigen::VectorXi& sub_indices, const Eigen::Array2f& sub_norm_limits);
                    Eigen::VectorXi indices; // control indices that form the subspace
                    Eigen::Array2f norm_limits;
                };
                /**
                     * Creates a new ramp velocity control space.
                     * @param velocity_limits - absolute maximal velocity for each dof
                     * @param acceleration_limits - absolute maximal acceleration for each dof
                     * @param duration_limits - duration limits (min, max)
                     */
                RampVelocityControlSpace(const ::ompl::base::StateSpacePtr& stateSpace,
                    const Eigen::VectorXf& velocity_limits,
                    const Eigen::VectorXf& acceleration_limits,
                    const Eigen::Array2f& duration_limits,
                    const std::vector<ControlSubspace>& sub_spaces = std::vector<ControlSubspace>());
                RampVelocityControlSpace(const ::ompl::base::StateSpacePtr& stateSpace,
                    const ControlLimits& limits,
                    const std::vector<ControlSubspace>& sub_spaces = std::vector<ControlSubspace>());
                ~RampVelocityControlSpace() override;

                /** ControlSpace */
                unsigned int getDimension() const override;
                ::ompl::control::Control* allocControl() const override;
                void freeControl(::ompl::control::Control* control) const override;
                void copyControl(::ompl::control::Control* control,
                    const ::ompl::control::Control* source) const override;
                bool equalControls(const ::ompl::control::Control* control_1,
                    const ::ompl::control::Control* control_2) const override;
                void nullControl(::ompl::control::Control* control) const override;
                ::ompl::control::ControlSamplerPtr allocDefaultControlSampler() const override;
                void printControl(const ::ompl::control::Control* control, std::ostream& out) const override;
                void printSettings(std::ostream& out) const override;
                void setup() override;
                unsigned int getNumParameters() const; // TODO this function is probably doing the same as getSerializationLength() is intended for
                // TODO not implemented
                unsigned int getSerializationLength() const override;
                // TODO not implemented
                void serialize(void* serialization, const ::ompl::control::Control* ctrl) const override;
                // TODO not implemented
                void deserialize(::ompl::control::Control* ctrl, const void* serialization) const override;
                bool isCompound() const override;
                /**
                     * NOT SUPPORTED. Throws a std::logic_error exception.
                     */
                double* getValueAddressAtIndex(::ompl::control::Control* control, unsigned int index) const override;

                /** RampVelocityControlSpace specific */
                void getAccelerationLimits(Eigen::VectorXf& limits) const;
                void getVelocityLimits(Eigen::VectorXf& limits) const;
                void getDurationLimits(Eigen::Array2f& limits) const;

                const RampVelocityControl* castControl(const ::ompl::control::Control* contrl) const;
                RampVelocityControl* castControl(::ompl::control::Control* contrl) const;

            protected:
                const std::vector<ControlSubspace>& getControlSubspaces() const;

            private:
                const Eigen::VectorXf _acceleration_limits;
                const Eigen::VectorXf _velocity_limits;
                const Eigen::Array2f _duration_limits;
                const std::vector<ControlSubspace> _sub_spaces;
            };
        }
    }
}
}

#endif //MANIPULATION_PLANNING_SUITE_RAMPVELOCITYCONTROL_H
