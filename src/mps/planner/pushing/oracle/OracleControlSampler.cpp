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
    _control_sampler = _si->allocControlSampler();
    _robot_state_sampler = _robot_state_space->allocStateSampler();
    _rng = mps::planner::util::random::getDefaultRandomGenerator();
}

OracleControlSampler::~OracleControlSampler()
{
}

void OracleControlSampler::samplePushingState(ompl::state::SimEnvWorldState* x_state,
    const ompl::state::SimEnvWorldState* x_prime_state,
    unsigned int tid,
    const float& p_uniform)
{
    static const std::string log_prefix("[mps::planner::pushing::oracle::OracleControlSampler::samplePushingState]");
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
    _pushing_oracle->samplePushingState(x_state, x_prime_state, tid, x_r);
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
    samplePushingState(x_state, x_prime_state_ompl, local_target_obj, p_uniform);
}

void OracleControlSampler::queryPolicy(::ompl::control::Control* control,
    const ompl::state::SimEnvWorldState* source, const ompl::state::SimEnvWorldState* dest, unsigned int obj_id, float p_uniform)
{
    static const std::string log_prefix("[mps::planner::pushing::oracle::OracleControlSampler::queryPolicy]");
    if (p_uniform > 0.0f) {
        float die = _rng->uniform01();
        if (die <= p_uniform) {
            mps_logging::logDebug("Sampling pushing action uniformly.", log_prefix);
            // sample a state uniformly
            _control_sampler->sample(control);
            return;
        }
    }
    _pushing_oracle->predictAction(source, dest, obj_id, control);
}

void OracleControlSampler::queryPolicy(::ompl::control::Control* control,
    const ::ompl::base::State* source, const ::ompl::base::State* dest, unsigned int obj_id, float p_uniform)
{
    static const std::string log_prefix("[mps::planner::pushing::oracle::OracleControlSampler::queryPolicy]");
    const auto* world_source = dynamic_cast<const mps_state::SimEnvWorldState*>(source);
    const auto* world_dest = dynamic_cast<const mps_state::SimEnvWorldState*>(dest);
    const auto* current_robot_state = world_source->getObjectState(_robot_id);
    const auto* current_obj_state = world_source->getObjectState(obj_id);
    const auto* dest_obj_state = world_dest->getObjectState(obj_id);
    queryPolicy(control, world_source, world_dest, obj_id, p_uniform);
}

bool OracleControlSampler::steerRobot(std::vector<::ompl::control::Control*>& controls,
    const ::ompl::base::State* source,
    const ::ompl::base::State* dest)
{
    auto x_state = dynamic_cast<const ompl::state::SimEnvWorldState*>(source);
    auto x_prime_state = dynamic_cast<const ompl::state::SimEnvWorldState*>(dest);
    return steerRobot(controls, x_state, x_prime_state);
}

bool OracleControlSampler::steerRobot(std::vector<::ompl::control::Control*>& controls,
    const mps::planner::ompl::state::SimEnvWorldState* source,
    const mps::planner::ompl::state::SimEnvWorldState* dest)
{
    auto* dest_robot_state = dest->getObjectState(_robot_id);
    return steerRobot(controls, source, dest_robot_state);
}

bool OracleControlSampler::steerRobot(std::vector<::ompl::control::Control*>& controls,
    const mps::planner::ompl::state::SimEnvWorldState* source,
    const mps::planner::ompl::state::SimEnvObjectState* dest)
{
    // TODO do we wanna enable this also for dynamic search spaces?
    _robot_oracle->steer(source, dest, controls);
    return not controls.empty();
}

// bool OracleControlSampler::steerPush(std::vector<::ompl::control::Control const*>& controls,
//     const ::ompl::base::State* source,
//     const ::ompl::base::State* dest,
//     unsigned int obj_id,
//     const float& action_noise)
// {
//     auto x_state = dynamic_cast<const ompl::state::SimEnvWorldState*>(source);
//     auto x_prime_state = dynamic_cast<const ompl::state::SimEnvWorldState*>(dest);
//     return steerPush(controls, x_state, x_prime_state, obj_id, action_noise);
// }

// bool OracleControlSampler::steerPush(std::vector<::ompl::control::Control const*>& controls,
//     const mps::planner::ompl::state::SimEnvWorldState* source,
//     const mps::planner::ompl::state::SimEnvWorldState* dest,
//     unsigned int object_id,
//     const float& action_noise)
// {
//     static const std::string log_prefix("[mps::planner::pushing::oracle::OracleControlSampler::steerPush]");
//     if (action_noise > 0.0f) { // if action noise is provided roll a die to decide whether to sample action uniformly
//         float die = _rng->uniform01();
//         if (die <= action_noise) {
//             randomControl(controls);
//             return true;
//         }
//     }
//     const auto* current_robot_state = source->getObjectState(_robot_id);
//     const auto* current_obj_state = source->getObjectState(object_id);
//     const auto* dest_obj_state = dest->getObjectState(object_id);
//     auto* control = _si->allocControl();
//     _pushing_oracle->predictAction(current_robot_state, current_obj_state, dest_obj_state,
//         object_id, control);
//     controls.push_back(control);
//     return true;
// }

void OracleControlSampler::randomControl(std::vector<::ompl::control::Control*>& controls)
{
    mps_logging::logDebug("Sampling random control", "[mps::planner::pushing::oracle::OracleControlSampler::randomControl]");
    auto* control = _si->allocControl();
    _control_sampler->sample(control);
    controls.push_back((control);
}

void OracleControlSampler::randomControl(::ompl::control::Control* control)
{
    _control_sampler->sample(control);
}
