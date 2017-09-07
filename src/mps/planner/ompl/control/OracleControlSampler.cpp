//
// Created by joshua on 8/14/17.
//
#include <mps/planner/ompl/control/OracleControlSampler.h>
#include <mps/planner/ompl/state/SimEnvState.h>
#include <mps/planner/util/Logging.h>
#include <mps/planner/ompl/control/Interfaces.h>

using namespace mps::planner::ompl::control;
namespace mps_oracle = mps::planner::pushing::oracle;
namespace mps_state = mps::planner::ompl::state;
namespace mps_logging = mps::planner::util::logging;

OracleControlSampler::Parameters::Parameters() {
    // TODO what are reasonable values here?
    min_feasibility = 0.05;
    min_pushability = 0.05;
}

OracleControlSampler::OracleControlSampler(::ompl::control::SpaceInformationPtr si,
                                           mps_oracle::PushingOraclePtr oracle,
                                           mps_oracle::RobotOraclePtr robot_orcale,
                                           const std::string& robot_name,
                                           const Parameters& params) :
//    ::ompl::control::DirectedControlSampler(si),
    _si(si),
    _pushing_oracle(oracle),
    _robot_oracle(robot_orcale),
    _control_idx(0),
    _params(params)
{
    auto state_space = std::dynamic_pointer_cast<mps_state::SimEnvWorldStateSpace>(_si->getStateSpace());
    if (!state_space) {
        throw std::logic_error("[mps::planner::ompl::control::OracleControlSampler]"
                               " Could not downcast state space to SimEnvWorldStateSpace."
                               " Provided state space not supported!");
    }
    int robot_id = state_space->getObjectIndex(robot_name);
    if (robot_id < 0) {
        throw std::logic_error("[mps::planner::ompl::control::OracleControlSampler]"
                                       "Could not find robot with name " + robot_name);
    }
    _robot_id = (unsigned int)(robot_id);
}

OracleControlSampler::~OracleControlSampler() = default;

void OracleControlSampler::sampleTo(std::vector<ompl::control::Control const*>& controls,
                                    const ompl::base::State *source,
                                    ompl::base::State *dest,
                                    unsigned int local_target_obj)
{
    auto* current_world_state = dynamic_cast<mps_state::SimEnvWorldState*>(source);
    auto* dest_world_state = dynamic_cast<mps_state::SimEnvWorldState*>(dest);
    if (!current_world_state or !dest_world_state) {
        throw std::logic_error("[mps::planner::ompl::control::OracleControlSampler]"
                               " Could not downcast source/dest state to SimEnvWorldState. Provided state not supported!");
    }
    if (local_target_obj == _robot_id) {
        steerRobot(controls, current_world_state, dest_world_state);
    } else {
        steerPush(controls, current_world_state, dest_world_state, local_target_obj);
    }
}

//unsigned int OracleControlSampler::sampleTo(::ompl::control::Control *control, const ::ompl::base::State *source,
//                                            ::ompl::base::State *dest) {
//    // TODO
//    return 0;
//}
//
//unsigned int OracleControlSampler::sampleTo(::ompl::control::Control *control, const ::ompl::control::Control *prev,
//                                            const ::ompl::base::State *source, ::ompl::base::State *dest) {
//    // TODO
//    return 0;
//}

void OracleControlSampler::steerRobot(std::vector<::ompl::control::Control const *> &controls,
                                      const mps::planner::ompl::state::SimEnvWorldState* source,
                                      mps::planner::ompl::state::SimEnvWorldState* dest)
{
    auto* current_robot_state = source->getObjectState(_robot_id);
    auto* dest_robot_state = dest->getObjectState(_robot_id);
    // TODO do we wanna enable this also for dynamic search spaces?
    Eigen::VectorXf current_config;
    Eigen::VectorXf dest_config;
    Eigen::VectorXf control_params;
    current_robot_state->getConfiguration(current_config);
    dest_robot_state->getConfiguration(dest_config);
    // TODO should we query it for multiple controls? In that case we would need to know what
    // TODO intermediate states we would have
    _robot_oracle->steer(current_config, dest_config, control_params);
    auto* control = getControl();
    control->setParameters(control_params);
}

void OracleControlSampler::steerPush(std::vector<::ompl::control::Control const *> &controls,
                                     const mps::planner::ompl::state::SimEnvWorldState* source,
                                     mps::planner::ompl::state::SimEnvWorldState* dest,
                                     unsigned int object_id)
{
    static const std::string log_prefix("[mps::planner::control::ompl::OracleControlSampler::steerPush]");
    auto* current_robot_state = source->getObjectState(_robot_id);
    auto* current_obj_state = source->getObjectState(object_id);
    auto* dest_robot_state = dest->getObjectState(_robot_id);
    auto* dest_obj_state = dest->getObjectState(object_id);
    // first ask the pushing oracle for pushability
    Eigen::VectorXf current_obj_config;
    current_obj_state->getConfiguration(current_obj_config);
    Eigen::VectorXf dest_obj_config;
    dest_obj_state->getConfiguration(dest_obj_config);
    float pushability = _pushing_oracle->predictPushability(current_obj_config, dest_obj_config);
    if (pushability < _params.min_pushability) {// bigger is better (1 / mahalanobis distance?)
        mps_logging::logDebug(boost::format("Pushability is only %f, sampling random action") %pushability,
                              log_prefix);
        randomAction(controls, source, dest, object_id); // the oracle can't help us. do random actions
    }
    // the oracle can help us at least a little bit
    mps_logging::logDebug("Sufficient pushability. Asking the pushing oracle for feasibility",
                          log_prefix);
    // ask for feasability
    Eigen::VectorXf current_robot_config;
    current_robot_state->getConfiguration(current_robot_config);
    Eigen::VectorXf dest_robot_config;
    dest_robot_state->getConfiguration(dest_robot_config);
    float feasibility = _pushing_oracle->predictFeasibility(current_robot_config, current_obj_config,
                                                           dest_obj_config);
    if (feasibility < _params.min_feasibility) { // if we have too low feasibility, steer robot to a better state
        mps_logging::logDebug(boost::format("Feasibility is only %f, steering robot") % feasibility,
                              log_prefix);
        Eigen::VectorXf new_robot_dest;
        _pushing_oracle->sampleFeasibleState(new_robot_dest, current_obj_config, dest_obj_config);
        // TODO create robot state from new_robot_dest
        steerRobot(controls, source, TODO);
    }
}

RealValueParameterizedControl* OracleControlSampler::getControl() {
    if (_control_idx >= _controls.size()) {
        _controls.push_back(dynamic_cast<RealValueParameterizedControl*>(_si->allocControl()));
    }
    return _controls[_control_idx++];
}

void OracleControlSampler::resetControlIdx() {
    _control_idx = 0;
}
