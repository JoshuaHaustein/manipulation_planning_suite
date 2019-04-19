//
// Created by joshua on 9/8/17.
//

#include <mps/planner/ompl/state/SimEnvState.h>
#include <mps/planner/pushing/oracle/RampComputer.h>
#include <mps/planner/util/Logging.h>

namespace mps_logging = mps::planner::util::logging;
using namespace mps::planner::pushing::oracle;

RampComputer::RampComputer(
    ompl::state::SimEnvObjectConfigurationSpacePtr robot_space,
    ompl::control::RampVelocityControlSpacePtr control_space,
    unsigned int robot_id)
    : _control_space(control_space)
    , _robot_space(robot_space)
    , _robot_id(robot_id)
{
    _ramp_control = dynamic_cast<ompl::control::RampVelocityControl*>(_control_space->allocControl());
}

RampComputer::RampComputer(const RampComputer& other, unsigned int robot_id)
    : _control_space(other._control_space)
    , _robot_space(other._robot_space)
    , _robot_id(robot_id)
{
    _ramp_control = dynamic_cast<ompl::control::RampVelocityControl*>(_control_space->allocControl());
}

mps::planner::pushing::oracle::RampComputer::~RampComputer()
{
    _control_space->freeControl(_ramp_control);
}

RampComputer& RampComputer::operator=(const RampComputer& other)
{
    _control_space->freeControl(_ramp_control);
    _control_space = other._control_space;
    _robot_space = other._robot_space;
    _ramp_control = dynamic_cast<ompl::control::RampVelocityControl*>(_control_space->allocControl());
    return *this;
}

void mps::planner::pushing::oracle::RampComputer::steer(const ompl::state::SimEnvObjectState* current_robot_state,
    const ompl::state::SimEnvObjectState* desired_robot_state,
    std::vector<::ompl::control::Control*>& controls) const
{
    std::vector<Eigen::VectorXf> control_params;
    steer(current_robot_state->getConfiguration(), desired_robot_state->getConfiguration(), controls);
}

void mps::planner::pushing::oracle::RampComputer::steer(
    const ompl::state::SimEnvWorldState* current_state,
    const ompl::state::SimEnvObjectState* desired_robot_state,
    std::vector<::ompl::control::Control*>& controls) const
{
    // we do not take the world state into account, so just forward it to the normal method
    steer(current_state->getObjectState(_robot_id)->getConfiguration(), desired_robot_state->getConfiguration(), controls);
}

void mps::planner::pushing::oracle::RampComputer::steer(const Eigen::VectorXf& current_robot_state,
    const Eigen::VectorXf& desired_robot_state,
    std::vector<::ompl::control::Control*>& controls) const
{
    std::vector<Eigen::VectorXf> control_params;
    compute_controls(current_robot_state, desired_robot_state, control_params);
    for (auto& control_param : control_params) {
        auto* ramp_control = dynamic_cast<mps::planner::ompl::control::RampVelocityControl*>(_control_space->allocControl());
        ramp_control->setParameters(control_param);
        controls.push_back(ramp_control);
    }
}

void mps::planner::pushing::oracle::RampComputer::steer(const Eigen::VectorXf& current_robot_state,
    const Eigen::VectorXf& desired_robot_state,
    ::ompl::control::Control* control) const
{
    std::vector<Eigen::VectorXf> control_params;
    compute_controls(current_robot_state, desired_robot_state, control_params);
    auto* ramp_control = dynamic_cast<mps::planner::ompl::control::RampVelocityControl*>(control);
    assert(ramp_control);
    if (control_params.empty()) {
        ramp_control->setZero();
    } else {
        ramp_control->setParameters(control_params.at(0));
    }
}

void mps::planner::pushing::oracle::RampComputer::compute_controls(const Eigen::VectorXf& current_robot_state,
    const Eigen::VectorXf& target_robot_state,
    std::vector<Eigen::VectorXf>& control_params) const
{
    static const std::string log_prefix("[mps::planner::pusing::oracle::RampComputer::steer");
    std::stringstream debug_ss;
    Eigen::VectorXf dir;
    _robot_space->computeDirection(current_robot_state, target_robot_state, dir);

    float distance = dir.norm();
    dir.normalize();

    debug_ss << "Direction for shortcut is " << dir.transpose();
    mps_logging::logDebug(debug_ss.str(), log_prefix);

    // Compute maximal v for the plateau phase
    Eigen::VectorXf vel_limits;
    Eigen::Array2f duration_limits;
    _control_space->getDurationLimits(duration_limits);
    _control_space->getVelocityLimits(vel_limits);
    float vabs = std::numeric_limits<float>::max();
    for (unsigned int i = 0; i < vel_limits.size(); ++i) {
        vabs = std::min(vabs, vel_limits[i] / std::abs(dir[i]));
    }
    Eigen::VectorXf vel = vabs * dir;
    debug_ss.str("");
    debug_ss << "Computed maximal velocity " << vel.transpose();
    mps_logging::logDebug(debug_ss.str(), log_prefix);
    _ramp_control->setMaxVelocities(vel, 0.0);
    float accel_time = _ramp_control->getAccelerationTime();
    mps_logging::logDebug(boost::format("Computed acceleration time %f") % accel_time, log_prefix);
    // Compute the distance that the manipulator travels only during the acceleration phase
    float dist_accel = (accel_time * vel).norm();

    while (distance > 0.0f) {
        if (dist_accel > distance) {
            // we would overshoot already, so choose a slower plateau velocity
            vabs = distance / accel_time; // accel_time * vabs = distance!
            vel = vabs * dir; // is slower than before; new acceleration time is < ta and thus we are not overshooting anymore
            _ramp_control->setMaxVelocities(vel, 0.0);
            accel_time = _ramp_control->getAccelerationTime();
            dist_accel = (accel_time * vel).norm();
        }

        distance = distance - dist_accel;
        float max_plateau_dist = std::min((vel * duration_limits[1]).norm(), distance);

        // Compute plateau time
        float duration = max_plateau_dist / vabs;
        _ramp_control->setMaxVelocities(vel, duration);
        control_params.push_back(_ramp_control->getParameters());
        distance = distance - max_plateau_dist;
    }
}