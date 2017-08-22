//
// Created by joshua on 8/14/17.
//
#include <mps/planner/ompl/control/RampVelocityControl.h>
#include <mps/planner/util/Random.h>

using namespace mps::planner::ompl::control;
namespace omc = ::ompl::control;

//////////////////////////////////////////////////////////////////////////////////////
////////////////////////////// RampVelocityControl ///////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////
RampVelocityControl::RampVelocityControl(RampVelocityControlSpaceConstPtr control_space):
        _control_space(control_space) {
}

RampVelocityControl::~RampVelocityControl() {
}

void RampVelocityControl::setMaxVelocities(const Eigen::VectorXf &max_velocities, float duration) {
    _max_velocities = max_velocities;
    _plateau_duration = duration;
    computeRamp();
}

void RampVelocityControl::setParameters(const Eigen::VectorXf& parameters) {
    assert(parameters.size() > 2);
    Eigen::VectorXf velocities;
    for (size_t i = 0; i < parameters.size() - 2; ++i) {
        velocities[i] = parameters[i];
    }
    float duration = parameters[parameters.size() - 2];
    setMaxVelocities(velocities, duration);
    setRestTime(parameters[parameters.size() - 1]);
}

Eigen::VectorXf RampVelocityControl::getParameters() const {
    Eigen::VectorXf params;
    getParameters(params);
    return params;
}

void RampVelocityControl::getParameters(Eigen::VectorXf& params) const {
    params.resize(_max_velocities.size() + 2);
    for (size_t i = 0; i < _max_velocities.size(); ++i) {
        params[i] = _max_velocities[i];
    }
    params[params.size() - 2] = _plateau_duration;
    params[params.size() - 1] = _rest_time;
}

Eigen::VectorXf RampVelocityControl::getMaxVelocities() const {
    return _max_velocities;
}

void RampVelocityControl::getMaxVelocities(Eigen::VectorXf& vel) const {
    vel = _max_velocities;
}

Eigen::VectorXf RampVelocityControl::getVelocity(float dt) const {
    Eigen::VectorXf vel;
    getVelocity(dt, vel);
    return vel;
}

void RampVelocityControl::getVelocity(float dt, Eigen::VectorXf& vel) const {
    if (dt < _acceleration_duration) { // acceleration phase
        vel = dt * _accelerations;
    } else if (dt < _acceleration_duration + _plateau_duration) { // plateau phase
        vel = _max_velocities;
    } else if (dt < 2.0 * _acceleration_duration + _plateau_duration) { // deceleration phase
        vel = _max_velocities - (dt - _acceleration_duration - _plateau_duration) * _accelerations;
    } else { // resting phase
        vel.resize(_max_velocities.size());
        vel.setZero();
    }
}

float RampVelocityControl::getMaxDuration() const {
    return 2.0f * _acceleration_duration + _plateau_duration + _rest_time;
}

float RampVelocityControl::getRestTime() const {
    return _rest_time;
}

void RampVelocityControl::addRestTime(float dt) {
    _rest_time += dt;
}

void RampVelocityControl::setRestTime(float t) {
    _rest_time = t;
}

void RampVelocityControl::computeRamp() {
    // first retrieve the acceleration limits
    RampVelocityControlSpaceConstPtr control_space = _control_space.lock();
    if (!control_space) {
        throw std::logic_error("[mps::planner::ompl::control::RampVelocityControl] Can not access control space.");
    }
    Eigen::VectorXf acceleration_limits;
    control_space->getAccelerationLimits(acceleration_limits);
    float max_accel_time = std::abs(_max_velocities[0] / acceleration_limits[0]);
    for (size_t i = 0; i < _max_velocities.size(); ++i) {
        max_accel_time = std::max(max_accel_time, std::abs(_max_velocities[i] / acceleration_limits[i]));
    }

    if (max_accel_time != 0.0) {
        _accelerations = 1.0f / max_accel_time * _max_velocities;
    } else {
        _accelerations.resize(_max_velocities.size());
        _accelerations.setZero();
    }
    _acceleration_duration = max_accel_time;
}

//////////////////////////////////////////////////////////////////////////////////////
////////////////////////////// RampVelocityControlSpace //////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////
RampVelocityControlSpace::RampVelocityControlSpace(const ::ompl::base::StateSpacePtr &stateSpace,
                                                   const Eigen::VectorXf &velocity_limits,
                                                   const Eigen::VectorXf &acceleration_limits,
                                                   const Eigen::Array2f &duration_limits) :
        omc::ControlSpace(stateSpace), _acceleration_limits(acceleration_limits),
        _velocity_limits(velocity_limits), _duration_limits(duration_limits) {
}

RampVelocityControlSpace::RampVelocityControlSpace(const ::ompl::base::StateSpacePtr &stateSpace,
                                                   const ControlLimits& limits) :
        RampVelocityControlSpace(stateSpace, limits.velocity_limits,
                                 limits.acceleration_limits, limits.duration_limits)
{
}

RampVelocityControlSpace::~RampVelocityControlSpace() {
}

unsigned int RampVelocityControlSpace::getDimension() const {
    // 1 dimension per dof velocity + duration of plateau (resting time is not part of the dimension)
    return (unsigned int) (_velocity_limits.size() + 1);
}

omc::Control* RampVelocityControlSpace::allocControl() const {
    return new RampVelocityControl(shared_from_this());
}

void RampVelocityControlSpace::freeControl(omc::Control* control) const {
    RampVelocityControl* ramp_control = castControl(control);
    delete ramp_control;
}

void RampVelocityControlSpace::copyControl(omc::Control* control,
                         const omc::Control* source) const {
    RampVelocityControl* ramp_control = castControl(control);
    const RampVelocityControl* source_control = castControl(source);
    ramp_control->setParameters(source_control->getParameters());
}

bool RampVelocityControlSpace::equalControls(const omc::Control* control_1,
                           const omc::Control* control_2) const {
    const RampVelocityControl* ramp_control_1 = castControl(control_1);
    const RampVelocityControl* ramp_control_2 = castControl(control_2);
    return (ramp_control_1->getParameters() - ramp_control_2->getParameters()).isZero();
}

void RampVelocityControlSpace::nullControl(omc::Control* control) const {
    RampVelocityControl* ramp_control = castControl(control);
    Eigen::VectorXf vel(getDimension());
    vel.setZero();
    ramp_control->setMaxVelocities(vel, 0.0f);
}

double* RampVelocityControlSpace::getValueAddressAtIndex(omc::Control* control, unsigned int index) const {
    throw std::logic_error("[mps::planner::ompl::control::getValueAddressAtIndex] Not supported.");
}

::ompl::control::ControlSamplerPtr RampVelocityControlSpace::allocDefaultControlSampler() const {
    return std::make_shared<RampVelocityControlSampler>(shared_from_this());
}

void RampVelocityControlSpace::printControl(const omc::Control *control, std::ostream &out) const {
    const RampVelocityControl* ramp_control = castControl(control);
    out << "RampVelocityControl: " << ramp_control->getParameters();
}

void RampVelocityControlSpace::printSettings(std::ostream &out) const {
    out << "printSettings is not supported.";
}

void RampVelocityControlSpace::setup() {
    omc::ControlSpace::setup();
}

unsigned int RampVelocityControlSpace::getSerializationLength() const {
    throw std::logic_error("[mps::planner::ompl::RampVelocityControlSapce::getSerializeationLength]"
                                   " Not implemented");
}

void RampVelocityControlSpace::serialize(void* serialization, const omc::Control* ctrl) const {
    throw std::logic_error("[mps::planner::ompl::RampVelocityControlSpace::serialize]"
                                   " Not implemented");
}

void RampVelocityControlSpace::deserialize(omc::Control* ctrl, const void* serialization) const {
    throw std::logic_error("[mps::planner::ompl::RampVelocityControlSpace::deserialize]"
                                   " Not implemented");
}

bool RampVelocityControlSpace::isCompound() const {
    return false;
}

void RampVelocityControlSpace::getAccelerationLimits(Eigen::VectorXf& limits) const {
    limits = _acceleration_limits;
}

void RampVelocityControlSpace::getVelocityLimits(Eigen::VectorXf& limits) const {
    limits = _velocity_limits;
}

void RampVelocityControlSpace::getDurationLimits(Eigen::Array2f& limits) const {
    limits = _duration_limits;
}

const RampVelocityControl* RampVelocityControlSpace::castControl(const omc::Control *control) const {
    const RampVelocityControl* ramp_control = dynamic_cast<const RampVelocityControl*>(control);
    if (!ramp_control) {
        throw std::logic_error("[mps::planner::ompl::control::RampVelocityControlSpace] Could not cast control."
                                       "Control type is not RampVelocityControl");
    }
    return ramp_control;
}

RampVelocityControl* RampVelocityControlSpace::castControl(omc::Control *control) const {
    RampVelocityControl* ramp_control = dynamic_cast<RampVelocityControl*>(control);
    if (!ramp_control) {
        throw std::logic_error("[mps::planner::ompl::control::RampVelocityControlSpace] Could not cast control."
                                       "Control type is not RampVelocityControl");
    }
    return ramp_control;
}

RampVelocityControlSampler::RampVelocityControlSampler(RampVelocityControlSpaceConstPtr control_space) :
        ControlSampler(control_space.get()), _control_space(control_space)
{
}

RampVelocityControlSampler::~RampVelocityControlSampler() {
}

void RampVelocityControlSampler::sample(omc::Control *control) {
    RampVelocityControlSpaceConstPtr control_space = _control_space.lock();
    if (!control_space) {
        throw std::logic_error("[mps::planner::ompl::control::RampVelocityControlSampler::sample]"
                                       "Invalid pointer. Could not access control space.");
    }
    RampVelocityControl* ramp_control = control_space->castControl(control);
    // sample values, first velocity
    Eigen::VectorXf velocity_limits;
    Eigen::VectorXf velocity;
    control_space->getVelocityLimits(velocity_limits);
    mps::planner::util::random::sampleUniform(-velocity_limits, velocity_limits, velocity);
    // now sample duration
    Eigen::Array2f duration_limits;
    control_space->getDurationLimits(duration_limits);
    ::ompl::RNGPtr rng = mps::planner::util::random::getDefaultRandomGenerator();
    float duration = (float)rng->uniformReal(duration_limits[0], duration_limits[1]);
    // let the ramp control adapt to these values and we are done
    ramp_control->setMaxVelocities(velocity, duration);
}
