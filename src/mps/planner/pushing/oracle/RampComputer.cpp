//
// Created by joshua on 9/8/17.
//

#include <mps/planner/pushing/oracle/RampComputer.h>
#include <mps/planner/ompl/state/SimEnvState.h>
#include <mps/planner/util/Logging.h>


namespace mps_logging = mps::planner::util::logging;
using namespace mps::planner::pushing::oracle;

RampComputer::RampComputer(
        ompl::state::SimEnvObjectConfigurationSpacePtr robot_space,
        ompl::control::RampVelocityControlSpacePtr control_space) :
        _robot_space(robot_space),
        _control_space(control_space)
{
    _ramp_control = dynamic_cast<ompl::control::RampVelocityControl*>(_control_space->allocControl());
}

mps::planner::pushing::oracle::RampComputer::~RampComputer() {
    _control_space->freeControl(_ramp_control);
}

void mps::planner::pushing::oracle::RampComputer::steer(const Eigen::VectorXf &current_robot_state,
                                                        const Eigen::VectorXf &desired_robot_state,
                                                        std::vector<Eigen::VectorXf> &control_params) const {
    static const std::string log_prefix("[mps::planner::pusing::oracle::RampComputer::steer");
    Eigen::VectorXf dir;
    _robot_space->computeDirection(current_robot_state, desired_robot_state, dir);

    float distance = dir.norm();
    dir.normalize();
    mps_logging::logDebug(boost::format("Direction for shortcut is ") << dir, log_prefix);

    // Compute maximal v for the plateau phase
    Eigen::VectorXf vel_limits;
    Eigen::Array2f duration_limits;
    _control_space->getDurationLimits(duration_limits);
    _control_space->getVelocityLimits(vel_limits);
    float vabs = std::numeric_limits<float>::max();
    for (unsigned int i = 0; i < vel_limits.size(); ++i) {
        vabs = std::min(vabs, vel_limits[i] / dir[i]);
    }
    Eigen::VectorXf vel = vabs * dir;
    mps_logging::logDebug(boost::format("Computed maximal velocity ") << vel, log_prefix);
    _ramp_control->setMaxVelocities(vel, 0.0);
    float accel_time = _ramp_control->getAccelerationTime();
    mps_logging::logDebug(boost::format("Computed acceleration time ") << accel_time, log_prefix);
    // Compute the distance that the manipulator travels only during the acceleration phase
    float dist_accel = (accel_time * vel).norm();

    while (distance > 0.0f) {
        if (dist_accel > distance) {
            // we would overshoot already, so choose a slower plateau velocity
            vabs = distance / accel_time;  // accel_time * vabs = distance!
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

