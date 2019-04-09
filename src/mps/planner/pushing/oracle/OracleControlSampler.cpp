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
    const std::string& robot_name)
    : //    ::ompl::control::DirectedControlSampler(si),
    _si(si)
    , _robot_oracle(robot_orcale)
    , _pushing_oracle(oracle)
    , _control_idx(0)
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
                               "Could not find robot with name "
            + robot_name);
    }
    _robot_id = (unsigned int)(robot_id);
    _robot_state_space = std::dynamic_pointer_cast<mps_state::SimEnvObjectStateSpace>(state_space->getSubspace(_robot_id));
    _robot_state = dynamic_cast<mps_state::SimEnvObjectState*>(_robot_state_space->allocState());
    Eigen::VectorXf vel = _robot_state->getConfiguration();
    vel.setZero();
    _robot_state->setVelocity(vel);
    assert(_robot_state);
    _control_sampler = _si->allocControlSampler();
    _robot_state_sampler = _robot_state_space->allocStateSampler();
    _rng = mps::planner::util::random::getDefaultRandomGenerator();
}

OracleControlSampler::~OracleControlSampler()
{
    _robot_state_space->freeState(_robot_state);
    for (auto* control : _controls) {
        _si->freeControl(control);
    }
}

void OracleControlSampler::sampleTo(std::vector<::ompl::control::Control const*>& controls,
    const ::ompl::base::State* source,
    const ::ompl::base::State* dest,
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

void OracleControlSampler::samplePushingState(::ompl::base::State* x_state_ompl,
    const ::ompl::base::State* x_prime_state_ompl,
    unsigned int local_target_obj,
    const float& p_uniform)
{
    static const std::string log_prefix("[mps::planner::pushing::oracle::OracleControlSampler::samplePushingState]");
    auto x_state = dynamic_cast<ompl::state::SimEnvWorldState*>(x_state_ompl);
    auto x_prime_state = dynamic_cast<const ompl::state::SimEnvWorldState*>(x_prime_state_ompl);
    assert(x_state);
    assert(x_prime_state);
    auto x_prime_t = x_prime_state->getObjectState(local_target_obj);
    auto x_t = x_state->getObjectState(local_target_obj);
    auto x_r = x_state->getObjectState(_robot_id);
    // with probability p_uniform sample feasible state uniformly
    if (p_uniform > 0.0f) {
        float die = _rng->uniform01();
        if (die <= p_uniform) {
            mps_logging::logDebug("Sampling pushing state uniformly.", log_prefix);
            // sample a state uniformly
            _robot_state_sampler->sampleUniform(x_r);
            return;
        }
    }
    mps_logging::logDebug("Sampling pushing state from oracle.", log_prefix);
    x_r->getConfiguration(_eigen_robot_state);
    x_t->getConfiguration(_eigen_current_object_state);
    x_prime_t->getConfiguration(_eigen_target_object_state);
    _pushing_oracle->samplePushingState(_eigen_current_object_state, _eigen_target_object_state, local_target_obj, _eigen_robot_state);
    // mps_logging::logDebug(boost::format("Sampled robot state %1% based on feasibility") % new_robot_dest.transpose(), log_prefix);
    // the oracle gave us a new robot state we should move to instead
    x_r->setConfiguration(_eigen_robot_state);
}

bool OracleControlSampler::steerRobot(std::vector<::ompl::control::Control const*>& controls,
    const ::ompl::base::State* source,
    const ::ompl::base::State* dest)
{
    auto x_state = dynamic_cast<const ompl::state::SimEnvWorldState*>(source);
    auto x_prime_state = dynamic_cast<const ompl::state::SimEnvWorldState*>(dest);
    return steerRobot(controls, x_state, x_prime_state);
}

bool OracleControlSampler::steerRobot(std::vector<::ompl::control::Control const*>& controls,
    const mps::planner::ompl::state::SimEnvWorldState* source,
    const mps::planner::ompl::state::SimEnvWorldState* dest)
{
    auto* dest_robot_state = dest->getObjectState(_robot_id);
    return steerRobot(controls, source, dest_robot_state);
}

bool OracleControlSampler::steerRobot(std::vector<::ompl::control::Control const*>& controls,
    const mps::planner::ompl::state::SimEnvWorldState* source,
    const mps::planner::ompl::state::SimEnvObjectState* dest)
{
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
    unsigned int obj_id,
    const float& action_noise)
{
    auto x_state = dynamic_cast<const ompl::state::SimEnvWorldState*>(source);
    auto x_prime_state = dynamic_cast<const ompl::state::SimEnvWorldState*>(dest);
    return steerPush(controls, x_state, x_prime_state, obj_id, action_noise);
}

bool OracleControlSampler::steerPush(std::vector<::ompl::control::Control const*>& controls,
    const mps::planner::ompl::state::SimEnvWorldState* source,
    const mps::planner::ompl::state::SimEnvWorldState* dest,
    unsigned int object_id,
    const float& action_noise)
{
    static const std::string log_prefix("[mps::planner::pushing::oracle::OracleControlSampler::steerPush]");
    if (action_noise > 0.0f) { // if action noise is provided roll a die to decide whether to sample action uniformly
        float die = _rng->uniform01();
        if (die <= action_noise) {
            randomControl(controls);
            return true;
        }
    }
    const auto* current_robot_state = source->getObjectState(_robot_id);
    const auto* current_obj_state = source->getObjectState(object_id);
    const auto* dest_obj_state = dest->getObjectState(object_id);
    current_obj_state->getConfiguration(_eigen_current_object_state);
    dest_obj_state->getConfiguration(_eigen_target_object_state);
    current_robot_state->getConfiguration(_eigen_robot_state);
    _pushing_oracle->predictAction(_eigen_robot_state, _eigen_current_object_state, _eigen_target_object_state,
        object_id, _eigen_control);
    auto* control = getControl();
    control->setParameters(_eigen_control);
    controls.push_back((::ompl::control::Control const*)control);
    mps_logging::logDebug(boost::format("The oracle suggested to take action %1%") % _eigen_control.transpose(), log_prefix);
    return true;
}

void OracleControlSampler::queryPolicy(::ompl::control::Control* control,
    const ::ompl::base::State* source, const ::ompl::base::State* dest, unsigned int obj_id)
{
    static const std::string log_prefix("[mps::planner::pushing::oracle::OracleControlSampler::queryPolicy]");
    const auto* world_source = dynamic_cast<const mps_state::SimEnvWorldState*>(source);
    const auto* world_dest = dynamic_cast<const mps_state::SimEnvWorldState*>(dest);
    const auto* current_robot_state = world_source->getObjectState(_robot_id);
    const auto* current_obj_state = world_source->getObjectState(obj_id);
    const auto* dest_obj_state = world_dest->getObjectState(obj_id);
    current_obj_state->getConfiguration(_eigen_current_object_state);
    dest_obj_state->getConfiguration(_eigen_target_object_state);
    current_robot_state->getConfiguration(_eigen_robot_state);
    _pushing_oracle->predictAction(_eigen_robot_state, _eigen_current_object_state, _eigen_target_object_state,
        obj_id, _eigen_control);
    auto* rv_control = dynamic_cast<mps_control::RealValueParameterizedControl*>(control);
    rv_control->setParameters(_eigen_control);
    mps_logging::logDebug(boost::format("The oracle suggested to take action %1%") % _eigen_control.transpose(), log_prefix);
}

void OracleControlSampler::randomControl(std::vector<::ompl::control::Control const*>& controls)
{
    mps_logging::logDebug("Sampling random control", "[mps::planner::pushing::oracle::OracleControlSampler::randomControl]");
    auto* control = getControl();
    _control_sampler->sample(control);
    controls.push_back((::ompl::control::Control const*)control);
}

mps_control::RealValueParameterizedControl* OracleControlSampler::getControl()
{
    if (_control_idx >= _controls.size()) {
        _controls.push_back(dynamic_cast<mps_control::RealValueParameterizedControl*>(_si->allocControl()));
    }
    return _controls[_control_idx++];
}

void OracleControlSampler::resetControlIdx()
{
    _control_idx = 0;
}
