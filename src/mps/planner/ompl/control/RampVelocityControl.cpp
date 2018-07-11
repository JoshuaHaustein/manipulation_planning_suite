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
    _rest_time = 0.0f;
    _plateau_duration = 0.0f;
    _acceleration_duration = 0.0f;
    _num_dofs = control_space->getDimension();
    assert(_num_dofs > 0);
}

RampVelocityControl::~RampVelocityControl() {
}

void RampVelocityControl::setMaxVelocities(const Eigen::VectorXf &max_velocities, float duration) {
    _max_velocities = max_velocities;
    _plateau_duration = duration;
    computeRamp();
}

void RampVelocityControl::setParameters(const Eigen::VectorXf& parameters) {
    assert(parameters.size() == _num_dofs + 1);
    Eigen::VectorXf velocities(_num_dofs - 1);
    for (size_t i = 0; i < velocities.size(); ++i) {
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
    params.resize(getNumParameters());
    for (size_t i = 0; i < _max_velocities.size(); ++i) {
        params[i] = _max_velocities[i];
    }
    params[params.size() - 2] = _plateau_duration;
    params[params.size() - 1] = _rest_time;
}

unsigned int RampVelocityControl::getNumParameters() const {
    return  _num_dofs + 1; // velocity_dofs + duration + waiting time
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

float RampVelocityControl::getAccelerationTime() const {
    return _acceleration_duration;
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

RampVelocityControlSpace::ControlSubspace::ControlSubspace(const Eigen::VectorXi &sub_indices,
                                                           const Eigen::Array2f &sub_norm_limits) :
    indices(sub_indices),
    norm_limits(sub_norm_limits)
{
}

RampVelocityControlSpace::ControlLimits::ControlLimits(const Eigen::VectorXf &velocity_limits,
                                                       const Eigen::VectorXf &acceleration_limits,
                                                       const Eigen::Array2f &duration_limits) :
        velocity_limits(velocity_limits),
        acceleration_limits(acceleration_limits),
        duration_limits(duration_limits)
{
}

RampVelocityControlSpace::ControlLimits::ControlLimits() = default;

RampVelocityControlSpace::RampVelocityControlSpace(const ::ompl::base::StateSpacePtr &stateSpace,
                                                   const Eigen::VectorXf &velocity_limits,
                                                   const Eigen::VectorXf &acceleration_limits,
                                                   const Eigen::Array2f &duration_limits,
                                                   const std::vector<ControlSubspace>& sub_spaces) :
        omc::ControlSpace(stateSpace), _acceleration_limits(acceleration_limits),
        _velocity_limits(velocity_limits), _duration_limits(duration_limits), _sub_spaces(sub_spaces) {
}

RampVelocityControlSpace::RampVelocityControlSpace(const ::ompl::base::StateSpacePtr &stateSpace,
                                                   const ControlLimits& limits,
                                                   const std::vector<ControlSubspace>& sub_spaces) :
        RampVelocityControlSpace(stateSpace, limits.velocity_limits,
                                 limits.acceleration_limits, limits.duration_limits, sub_spaces)
{
}

RampVelocityControlSpace::~RampVelocityControlSpace() = default;

unsigned int RampVelocityControlSpace::getDimension() const {
    // 1 dimension per dof velocity + duration of plateau (resting time is not part of the dimension)
    return (unsigned int) (_velocity_limits.rows() + 1);
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
    Eigen::VectorXf vel(_velocity_limits.rows());
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

// TODO this function is probably doing the same as getSerializationLength() is intended for
unsigned int RampVelocityControlSpace::getNumParameters() const {
    return getDimension() + 1; // includes resting time
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

const std::vector<RampVelocityControlSpace::ControlSubspace>& RampVelocityControlSpace::getControlSubspaces() const
{
    return _sub_spaces;
}

RampVelocityControlSampler::RampVelocityControlSampler(RampVelocityControlSpaceConstPtr control_space) :
        ControlSampler(control_space.get()), _control_space(control_space)
{
    _values_buffer = new double[control_space->getDimension()];
}

RampVelocityControlSampler::~RampVelocityControlSampler() {
    delete[] _values_buffer;
}

void RampVelocityControlSampler::sample(omc::Control *control) {
    RampVelocityControlSpaceConstPtr control_space = _control_space.lock();
    if (!control_space) {
        throw std::logic_error("[mps::planner::ompl::control::RampVelocityControlSampler::sample]"
                                       "Invalid pointer. Could not access control space.");
    }
    auto rng = mps::planner::util::random::getDefaultRandomGenerator();
    RampVelocityControl* ramp_control = control_space->castControl(control);
    // sample values, first velocity
    Eigen::VectorXf velocity_limits;
    control_space->getVelocityLimits(velocity_limits);
    long num_dofs = velocity_limits.rows();
    assert(num_dofs > 0);
    Eigen::VectorXf velocity(num_dofs);

    // we might have control subspaces for which we want to sample jointly from a ball
    std::vector<bool> index_sampled((unsigned long)num_dofs, false);
    auto& subspaces = control_space->getControlSubspaces();
    // run over all subspaces
    for (auto& subspace : subspaces) {
        auto subspace_dim = (unsigned int)subspace.indices.size();
        assert(subspace_dim <= num_dofs);
        rng->uniformInBall(subspace.norm_limits[1], subspace_dim, _values_buffer);
        for (long i = 0; i < subspace_dim; ++i) {
            auto idx = subspace.indices[i];
            velocity[idx] = _values_buffer[i];
            index_sampled[idx] = true;
        }
    }
    for (unsigned int i = 0; i < index_sampled.size(); ++i) {
        if (not index_sampled.at(i)) {
            velocity[i] = rng->uniformReal(-velocity_limits[i], velocity_limits[i]);
        }
    }
    // now sample duration
    Eigen::Array2f duration_limits;
    control_space->getDurationLimits(duration_limits);
    auto duration = (float)rng->uniformReal(duration_limits[0], duration_limits[1]);
    // let the ramp control adapt to these values and we are done
    ramp_control->setMaxVelocities(velocity, duration);
}
