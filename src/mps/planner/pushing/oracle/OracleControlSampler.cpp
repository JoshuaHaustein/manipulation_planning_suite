//
// Created by joshua on 8/14/17.
//
#include <mps/planner/pushing/oracle/OracleControlSampler.h>
#include <mps/planner/util/Logging.h>

using namespace mps::planner::pushing::oracle;
namespace mps_state = mps::planner::ompl::state;
namespace mps_control = mps::planner::ompl::control;
namespace mps_logging = mps::planner::util::logging;

OracleControlSampler::OracleControlSampler(::ompl::control::SpaceInformationPtr si,
                                           PushingOraclePtr oracle,
                                           RobotOraclePtr robot_orcale,
                                           const std::string& robot_name) :
//    ::ompl::control::DirectedControlSampler(si),
    _si(si),
    _robot_oracle(robot_orcale),
    _pushing_oracle(oracle),
    _control_idx(0)
{
    auto state_space = std::dynamic_pointer_cast<mps_state::SimEnvWorldStateSpace>(_si->getStateSpace());
    if (!state_space) {
        throw std::logic_error("[mps::planner::pushing::oracle::OracleControlSampler]"
                               " Could not downcast state space to SimEnvWorldStateSpace."
                               " Provided state space not supported!");
    }
    int robot_id = state_space->getObjectIndex(robot_name);
    if (robot_id < 0) {
        throw std::logic_error("[mps::planner::pushing::oracle::OracleControlSampler]"
                                       "Could not find robot with name " + robot_name);
    }
    _robot_id = (unsigned int)(robot_id);
    _robot_state_space = std::dynamic_pointer_cast<mps_state::SimEnvObjectStateSpace>(state_space->getSubspace(_robot_id));
    _robot_state = dynamic_cast<mps_state::SimEnvObjectState*>(_robot_state_space->allocState());
    Eigen::VectorXf vel = _robot_state->getConfiguration();
    vel.setZero();
    _robot_state->setVelocity(vel);
    assert(_robot_state);
    _control_sampler = _si->allocControlSampler();
}

OracleControlSampler::~OracleControlSampler() {
    _robot_state_space->freeState(_robot_state);
    for (auto* control : _controls) {
        _si->freeControl(control);
    }
}

void OracleControlSampler::sampleTo(std::vector<::ompl::control::Control const*>& controls,
                                    const ::ompl::base::State *source,
                                    const ::ompl::base::State *dest,
                                    unsigned int local_target_obj)
{
    auto* current_world_state = dynamic_cast<mps_state::SimEnvWorldState const*>(source);
    auto* dest_world_state = dynamic_cast<mps_state::SimEnvWorldState const*>(dest);
    if (!current_world_state or !dest_world_state) {
        throw std::logic_error("[mps::planner::pushing::oracle::OracleControlSampler]"
                               " Could not downcast source/dest state to SimEnvWorldState. Provided state not supported!");
    }
    // first reset our control cache
    resetControlIdx();
    // next choose whether we wanna steer the robot or push an object
    bool has_control = false;
    if (local_target_obj == _robot_id) {
        has_control = steerRobot(controls, current_world_state, dest_world_state);
    } else {
        has_control = steerPush(controls, current_world_state, dest_world_state, local_target_obj);
    }
    if (not has_control) {
        randomControl(controls);
    }
}

void OracleControlSampler::sampleFeasibleState(::ompl::base::State* x_state_ompl,
                                               const ::ompl::base::State* x_prime_state_ompl,
                                               unsigned int local_target_obj)
{
    static const std::string log_prefix("[mps::planner::pushing::oracle::OracleControlSampler::sampleFeasibleState]");
    auto x_state = dynamic_cast<ompl::state::SimEnvWorldState*>(x_state_ompl);
    auto x_prime_state = dynamic_cast<const ompl::state::SimEnvWorldState*>(x_prime_state_ompl);
    assert(x_state);
    assert(x_prime_state);
    auto x_prime_t = x_prime_state->getObjectState(local_target_obj);
    auto x_t = x_state->getObjectState(local_target_obj);
    auto x_r = x_state->getObjectState(_robot_id);
    Eigen::VectorXf new_robot_dest;
    x_r->getConfiguration(new_robot_dest);
    Eigen::VectorXf current_obj_state;
    x_t->getConfiguration(current_obj_state);
    Eigen::VectorXf target_obj_state;
    x_prime_t->getConfiguration(target_obj_state);
    _pushing_oracle->sampleFeasibleState(current_obj_state, target_obj_state, local_target_obj, new_robot_dest);
    // mps_logging::logDebug(boost::format("Sampled robot state %1% based on feasibility") % new_robot_dest.transpose(), log_prefix);
    // the oracle gave us a new robot state we should move to instead
    x_r->setConfiguration(new_robot_dest);
}

float OracleControlSampler::getFeasibility(const ::ompl::base::State* x_state_ompl,
                                           const ::ompl::base::State* x_prime_state_ompl,
                                           unsigned int active_obj_id) const
{
    auto x_state = dynamic_cast<const ompl::state::SimEnvWorldState*>(x_state_ompl);
    auto x_prime_state = dynamic_cast<const ompl::state::SimEnvWorldState*>(x_prime_state_ompl);
    assert(x_state);
    assert(x_prime_state);
    auto x_prime_t = x_prime_state->getObjectState(active_obj_id);
    auto x_t = x_state->getObjectState(active_obj_id);
    auto x_r = x_state->getObjectState(_robot_id);
    Eigen::VectorXf new_robot_dest;
    x_r->getConfiguration(new_robot_dest);
    Eigen::VectorXf current_obj_state;
    x_t->getConfiguration(current_obj_state);
    Eigen::VectorXf target_obj_state;
    x_prime_t->getConfiguration(target_obj_state);
    return _pushing_oracle->predictFeasibility(new_robot_dest, current_obj_state, target_obj_state, active_obj_id);
}

bool OracleControlSampler::steerRobot(std::vector<::ompl::control::Control const *> &controls,
                                      const ::ompl::base::State *source,
                                      const ::ompl::base::State *dest)
{
    auto x_state = dynamic_cast<const ompl::state::SimEnvWorldState*>(source);
    auto x_prime_state = dynamic_cast<const ompl::state::SimEnvWorldState*>(dest);
    return steerRobot(controls, x_state, x_prime_state);
}

bool OracleControlSampler::steerRobot(std::vector<::ompl::control::Control const *> &controls,
                                      const mps::planner::ompl::state::SimEnvWorldState* source,
                                      const mps::planner::ompl::state::SimEnvWorldState* dest)
{
    auto* dest_robot_state = dest->getObjectState(_robot_id);
    return steerRobot(controls, source, dest_robot_state);
}

bool OracleControlSampler::steerRobot(std::vector<::ompl::control::Control const*>& controls,
                const mps::planner::ompl::state::SimEnvWorldState* source,
                const mps::planner::ompl::state::SimEnvObjectState* dest) {
    const auto* current_robot_state = source->getObjectState(_robot_id);
    // TODO do we wanna enable this also for dynamic search spaces?
    std::vector<Eigen::VectorXf> control_params;
    _robot_oracle->steer(current_robot_state, dest, source, control_params);
    for (auto& control_param : control_params) {
        auto* control = getControl();
        control->setParameters(control_param);
        controls.push_back((::ompl::control::Control const*)control);
    }
    return not controls.empty();
}

bool OracleControlSampler::steerPush(std::vector<::ompl::control::Control const*>& controls,
                                     const ::ompl::base::State* source,
                                     const ::ompl::base::State* dest,
                                     unsigned int obj_id)
{
    auto x_state = dynamic_cast<const ompl::state::SimEnvWorldState*>(source);
    auto x_prime_state = dynamic_cast<const ompl::state::SimEnvWorldState*>(dest);
    return steerPush(controls, x_state, x_prime_state, obj_id);
}

bool OracleControlSampler::steerPush(std::vector<::ompl::control::Control const *> &controls,
                                     const mps::planner::ompl::state::SimEnvWorldState* source,
                                     const mps::planner::ompl::state::SimEnvWorldState* dest,
                                     unsigned int object_id)
{
    static const std::string log_prefix("[mps::planner::pushing::oracle::OracleControlSampler::steerPush]");
    const auto* current_robot_state = source->getObjectState(_robot_id);
    const auto* current_obj_state = source->getObjectState(object_id);
    const auto* dest_obj_state = dest->getObjectState(object_id);
    Eigen::VectorXf current_obj_config;
    current_obj_state->getConfiguration(current_obj_config);
    Eigen::VectorXf dest_obj_config;
    dest_obj_state->getConfiguration(dest_obj_config);
    Eigen::VectorXf current_robot_config;
    current_robot_state->getConfiguration(current_robot_config);
    Eigen::VectorXf control_param;
    _pushing_oracle->predictAction(current_robot_config, current_obj_config, dest_obj_config,
                                   object_id, control_param);
    auto* control = getControl();
    control->setParameters(control_param);
    controls.push_back((::ompl::control::Control const*)control);
    mps_logging::logDebug(boost::format("The oracle suggested to take action %1%") % control_param.transpose(), log_prefix);
    return true;
}

// bool OracleControlSampler::steerPushSimple(std::vector<::ompl::control::Control const*>& controls,
//                                            const ::ompl::base::State* source,
//                                            const ::ompl::base::State* dest,
//                                            unsigned int obj_id)
// {
//     auto x_state = dynamic_cast<const ompl::state::SimEnvWorldState*>(source);
//     auto x_prime_state = dynamic_cast<const ompl::state::SimEnvWorldState*>(dest);
//     return steerPushSimple(controls, x_state, x_prime_state, obj_id);
// }

// bool OracleControlSampler::steerPushSimple(std::vector<::ompl::control::Control const *> &controls,
//                                      const mps::planner::ompl::state::SimEnvWorldState* source,
//                                      const mps::planner::ompl::state::SimEnvWorldState* dest,
//                                      unsigned int object_id)
// {
//     static const std::string log_prefix("[mps::planner::pushing::oracle::OracleControlSampler::steerPushSimple]");
//     const auto* current_robot_state = source->getObjectState(_robot_id);
//     const auto* current_obj_state = source->getObjectState(object_id);
//     const auto* dest_obj_state = dest->getObjectState(object_id);
//     Eigen::VectorXf current_obj_config;
//     current_obj_state->getConfiguration(current_obj_config);
//     Eigen::VectorXf dest_obj_config;
//     dest_obj_state->getConfiguration(dest_obj_config);
//     Eigen::VectorXf current_robot_config;
//     current_robot_state->getConfiguration(current_robot_config);
//     // TODO needs to be evaluated. Should we really do this projection?
//     mps_logging::logDebug(boost::format("Projecting object state %1% to pushability distribution") % dest_obj_config.transpose(), log_prefix);
//     _pushing_oracle->projectToPushability(current_obj_config, dest_obj_config, _params.min_pushability, object_id, dest_obj_config);
//     mps_logging::logDebug(boost::format("Projected state to %1%") % dest_obj_config.transpose(), log_prefix);
//     // TODO do not need to actually compute pushability
//     float pushability = _pushing_oracle->predictPushability(current_obj_config, dest_obj_config, object_id);
//     mps_logging::logDebug(boost::format("New pushability is %f") % pushability, log_prefix);
//     // ask for feasibility
//     float feasibility = _pushing_oracle->predictFeasibility(current_robot_config, current_obj_config,
//                                                             dest_obj_config, object_id);
//     // if we have too low feasibility, do a random action, i.e. we emulate having a well defined distribution
//     if (feasibility < _params.min_feasibility) {
//         mps_logging::logDebug(boost::format("Feasibility is only %f, steering robot") % feasibility,
//                               log_prefix);
//         Eigen::VectorXf new_robot_dest(_robot_state->getConfiguration());
//         _pushing_oracle->sampleFeasibleState(current_obj_config, dest_obj_config, object_id, new_robot_dest);
//         mps_logging::logDebug(boost::format("Sampled robot state %1% based on feasibility") % new_robot_dest.transpose(), log_prefix);
//         // the oracle gave us a new robot state we should move to instead
//         _robot_state->setConfiguration(new_robot_dest);
//         randomControl(controls, source, dest, object_id);
//         return true;
//     }
//     // The oracle is confident it can help
//     mps_logging::logDebug("The oracle is confident it can help.", log_prefix);
//     Eigen::VectorXf control_param;
//     _pushing_oracle->predictAction(current_robot_config, current_obj_config, dest_obj_config,
//                                    object_id, control_param);
//     auto* control = getControl();
//     control->setParameters(control_param);
//     controls.push_back((::ompl::control::Control const*)control);
//     mps_logging::logDebug(boost::format("The oracle suggested to take action %1%") % control_param.transpose(), log_prefix);
//     return true;
// }

void OracleControlSampler::randomControl(std::vector<::ompl::control::Control const *> &controls)
{
    mps_logging::logDebug("Sampling random control", "[mps::planner::pushing::oracle::OracleControlSampler::randomControl]");
    auto* control = getControl();
    _control_sampler->sample(control);
    controls.push_back((::ompl::control::Control const*)control);
}

mps_control::RealValueParameterizedControl* OracleControlSampler::getControl() {
    if (_control_idx >= _controls.size()) {
        _controls.push_back(dynamic_cast<mps_control::RealValueParameterizedControl*>(_si->allocControl()));
    }
    return _controls[_control_idx++];
}

void OracleControlSampler::resetControlIdx() {
    _control_idx = 0;
}

