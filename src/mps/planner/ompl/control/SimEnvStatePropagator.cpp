//
// Created by joshua on 8/21/17.
//

#include <mps/planner/ompl/control/Interfaces.h>
#include <mps/planner/ompl/control/SimEnvStatePropagator.h>
#include <mps/planner/ompl/state/SimEnvState.h>
#include <mps/planner/util/Logging.h>

using namespace mps::planner::ompl::control;
using namespace mps::planner::ompl::state;
namespace mps_logging = mps::planner::util::logging;

SimEnvStatePropagator::SimEnvStatePropagator(::ompl::control::SpaceInformationPtr si,
    sim_env::WorldPtr world,
    sim_env::RobotControllerPtr controller,
    bool semi_dynamic,
    float t_max)
    : ::ompl::control::StatePropagator(si)
    , _world(world)
    , _controller(controller)
    , _semi_dynamic(semi_dynamic)
    , _t_max(t_max)
{
    _state_space = std::dynamic_pointer_cast<state::SimEnvWorldStateSpace>(si->getStateSpace());
    // _validity_checker = std::make_shared<state::SimEnvValidityChecker>(si, _world);
    // _validity_checker->collision_policy = collision_policy;
    _validity_checker = std::dynamic_pointer_cast<state::SimEnvValidityChecker>(si->getStateValidityChecker());
    assert(_validity_checker);
    assert(not _state_space.expired());
}

SimEnvStatePropagator::~SimEnvStatePropagator()
{
}

void SimEnvStatePropagator::propagate(const ::ompl::base::State* state, const ::ompl::control::Control* control,
    double duration, ::ompl::base::State* result) const
{
}

bool SimEnvStatePropagator::propagate(const ::ompl::base::State* state, ::ompl::control::Control* control,
    ::ompl::base::State* result) const
{

    static const std::string log_prefix("[mps::planner::ompl::control::SimEnvStatePropagator::propagate]");
    mps_logging::logDebug("Propagating state", log_prefix);
    auto state_space = _state_space.lock();
    if (!state_space) {
        throw std::logic_error("[mps::planner::ompl::control::SimEnvStatePropagator::propagate]"
                               "Could not acquire access to state space. Weak pointer is invalid.");
    }
    auto* world_state = state->as<state::SimEnvWorldState>();
    auto* result_world_state = result->as<state::SimEnvWorldState>();
    auto* timed_control = dynamic_cast<control::TimedControl*>(control);
    assert(timed_control);
    // lock the world for the whole propagation
    std::lock_guard<std::recursive_mutex> world_lock(_world->getMutex());
    // save current world state
    _world->saveState();
    // set the world to the start state
    state_space->setToState(_world, world_state);
    // reset resting time in case we have a semi-dynamic action
    if (_semi_dynamic) {
        auto* semi_dyn_control = dynamic_cast<control::SemiDynamicControl*>(control);
        assert(semi_dyn_control);
        semi_dyn_control->setRestDuration(0.0f);
    }
    // now propagate for the duration of action
    float time = 0.0f;
    bool propagation_success = true;
    assert(_world->getPhysicsTimeStep() > 0.0f);
    Eigen::VectorXf target(_controller->getTargetDimension());
    std::vector<sim_env::Contact> contacts;
    while (time < timed_control->getDuration() and propagation_success) {
        timed_control->getTarget(time, target);
        _controller->setTarget(target);
        _world->stepPhysics(contacts);
        time += _world->getPhysicsTimeStep();
        propagation_success = _validity_checker->isValidIntermediate(contacts);
        contacts.clear();
    }

    // in case we have a semi dynamic propagation, continue propagating until rest
    if (_semi_dynamic and propagation_success) {
        //        auto* semi_dyn_control = velocity_control->as<control::SemiDynamicVelocityControl>();
        auto* semi_dyn_control = dynamic_cast<control::SemiDynamicControl*>(control);
        assert(semi_dyn_control);
        float resting_time = 0.0f;
        // in case we have a velocity control, set target velocity to zero
        auto* velocity_control = dynamic_cast<control::VelocityControl*>(control);
        if (velocity_control) {
            target.setZero();
            _controller->setTarget(target); // constant zero target velocity
        }
        // propagate until either t_max is reached, or the world is at rest
        while (resting_time <= _t_max and not _world->atRest() and propagation_success) {
            _world->stepPhysics(contacts);
            resting_time += _world->getPhysicsTimeStep();
            propagation_success = _validity_checker->isValidIntermediate(contacts);
            contacts.clear();
        }
        semi_dyn_control->setRestDuration(resting_time);
        propagation_success = resting_time <= _t_max and _world->atRest() and propagation_success;
        //        _world->getLogger()->logDebug(boost::format("Semi-dynamic propagation terminated. Resting time is: %f")
        //                                      % resting_time,
        //                                      log_prefix);
    }

    state_space->extractState(_world, result_world_state);
    propagation_success = propagation_success and _validity_checker->isValid(result_world_state); // TODO isValid produces some unnecessary overhead here
    _world->restoreState();
    // TODO there are currently no validity checks performed here, i.e. is the outcoming state physicaly feasible
    // TODO (remember squeezing issue).
    mps_logging::logDebug(boost::format("Propagation finished. Success: %1%") % propagation_success,
        log_prefix);
    std::stringstream ss;
    result_world_state->print(ss);
    mps_logging::logDebug("Resulting state: " + ss.str(), log_prefix);
    return propagation_success;
}

bool SimEnvStatePropagator::canPropagateBackward() const
{
    return false;
}
