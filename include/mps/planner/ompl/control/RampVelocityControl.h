//
// Created by joshua on 8/14/17.
//

#ifndef MANIPULATION_PLANNING_SUITE_RAMPVELOCITYCONTROL_H
#define MANIPULATION_PLANNING_SUITE_RAMPVELOCITYCONTROL_H

#include <mps/planner/ompl/control/Interfaces.h>
#include <ompl/control/ControlSpace.h>
#include <memory>

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
                    void setParameters(const Eigen::VectorXf& parameters);
                    virtual Eigen::VectorXf getParameters() const;
                    virtual void getParameters(Eigen::VectorXf& params) const;

                    /**
                     * Returns the maximal velocities set for this action.
                     * @return
                     */
                    Eigen::VectorXf getMaxVelocities() const;
                    void getMaxVelocities(Eigen::VectorXf& vel) const;

                    /** VelocityProfile interface **/
                    virtual Eigen::VectorXf getVelocity(float dt) const;
                    virtual void getVelocity(float dt, Eigen::VectorXf& vel) const;
                    virtual float getMaxDuration() const;

                    /** FiniteRestingVelocityControl interface **/
                    virtual float getRestTime() const;
                    virtual void addRestTime(float dt);
                    virtual void setRestTime(float t);

                private:
                    RampVelocityControlSpaceConstWeakPtr _control_space;
                    Eigen::VectorXf _accelerations;
                    Eigen::VectorXf _max_velocities;
                    float _plateau_duration;
                    float _acceleration_duration;
                    float _rest_time;

                    void computeRamp();
                };

                class RampVelocityControlSpace : public ::ompl::control::ControlSpace,
                                                 public std::enable_shared_from_this<RampVelocityControlSpace>{
                public:
                    struct ControlLimits {
                        Eigen::VectorXf velocity_limits;
                        Eigen::VectorXf acceleration_limits;
                        Eigen::Array2f duration_limits;
                        ControlLimits(const Eigen::VectorXf& velocity_limits,
                                      const Eigen::VectorXf& acceleration_limits,
                                      const Eigen::Array2f& duration_limits);
                    };
                    /**
                     * Creates a new ramp velocity control space.
                     * @param velocity_limits - absolute maximal velocity for each dof
                     * @param acceleration_limits - absolute maximal acceleration for each dof
                     * @param duration_limits - duration limits (min, max)
                     */
                    RampVelocityControlSpace(const ::ompl::base::StateSpacePtr &stateSpace,
                                             const Eigen::VectorXf &velocity_limits,
                                             const Eigen::VectorXf &acceleration_limits,
                                             const Eigen::Array2f &duration_limits);
                    RampVelocityControlSpace(const ::ompl::base::StateSpacePtr &stateSpace,
                                             const ControlLimits& limits);
                    ~RampVelocityControlSpace();

                    /** ControlSpace */
                    virtual unsigned int getDimension() const override;
                    virtual ::ompl::control::Control* allocControl() const override;
                    virtual void freeControl(::ompl::control::Control* control) const override;
                    virtual void copyControl(::ompl::control::Control* control,
                                             const ::ompl::control::Control* source) const override;
                    virtual bool equalControls(const ::ompl::control::Control* control_1,
                                               const ::ompl::control::Control* control_2) const override;
                    virtual void nullControl(::ompl::control::Control* control) const override;
                    virtual ::ompl::control::ControlSamplerPtr allocDefaultControlSampler() const override;
                    virtual void printControl(const ::ompl::control::Control* control, std::ostream& out) const override;
                    virtual void printSettings(std::ostream& out) const override;
                    virtual void setup() override;
                    // TODO not implemented
                    virtual unsigned int getSerializationLength() const override;
                    // TODO not implemented
                    virtual void serialize(void* serialization, const ::ompl::control::Control* ctrl) const override;
                    // TODO not implemented
                    virtual void deserialize(::ompl::control::Control* ctrl, const void *serialization) const override;
                    virtual bool isCompound() const override;
                    /**
                     * NOT SUPPORTED. Throws a std::logic_error exception.
                     */
                    virtual double* getValueAddressAtIndex(::ompl::control::Control* control, unsigned int index) const override;

                    /** RampVelocityControlSpace specific */
                    void getAccelerationLimits(Eigen::VectorXf& limits) const;
                    void getVelocityLimits(Eigen::VectorXf& limits) const;
                    void getDurationLimits(Eigen::Array2f& limits) const;

                    const RampVelocityControl* castControl(const ::ompl::control::Control* contrl) const;
                    RampVelocityControl* castControl(::ompl::control::Control* contrl) const;
                private:
                    const Eigen::VectorXf _acceleration_limits;
                    const Eigen::VectorXf _velocity_limits;
                    const Eigen::Array2f _duration_limits;
                };

                class RampVelocityControlSampler : public ::ompl::control::ControlSampler {
                public:
                    RampVelocityControlSampler(RampVelocityControlSpaceConstPtr control_space);
                    ~RampVelocityControlSampler();

                    virtual void sample(::ompl::control::Control* control) override;
                private:
                    const RampVelocityControlSpaceConstWeakPtr _control_space;

                };
            }
        }
    }
}

#endif //MANIPULATION_PLANNING_SUITE_RAMPVELOCITYCONTROL_H
