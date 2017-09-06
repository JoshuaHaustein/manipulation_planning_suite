//
// Created by joshua on 8/21/17.
//

#include <mps/planner/ompl/control/SimEnvStatePropagator.h>
#include <mps/planner/ompl/control/Interfaces.h>
#include <mps/planner/ompl/state/SimEnvState.h>
#include <mps/planner/ompl/control/Interfaces.h>

using namespace mps::planner::ompl::control;
using namespace mps::planner::ompl::state;

SimEnvStatePropagator::SimEnvStatePropagator(::ompl::control::SpaceInformationPtr si,
                                             sim_env::WorldPtr world,
                                             sim_env::RobotVelocityControllerPtr controller,
                                             bool semi_dynamic,
                                             float t_max) :
        ::ompl::control::StatePropagator(si),
        _world(world),
        _controller(controller),
        _semi_dynamic(semi_dynamic),
        _t_max(t_max)
{
    _state_space = std::dynamic_pointer_cast<state::SimEnvWorldStateSpace>(si->getStateSpace());
    assert(not _state_space.expired());
}

SimEnvStatePropagator::~SimEnvStatePropagator() {
}

void SimEnvStatePropagator::propagate(const ::ompl::base::State* state, const ::ompl::control::Control* control,
                                      double duration, ::ompl::base::State* result) const {

}


bool SimEnvStatePropagator::propagate(const ::ompl::base::State* state, ::ompl::control::Control* control,
                                      ::ompl::base::State* result) const {

    static const std::string log_prefix("[mps::planner::ompl::control::SimEnvStatePropagator::propagate]");
    auto logger = _world->getLogger();
    logger->logDebug("Propagating state", log_prefix);
    auto state_space = _state_space.lock();
    if (!state_space) {
        throw std::logic_error("[mps::planner::ompl::control::SimEnvStatePropagator::propagate]"
                                       "Could not acquire access to state space. Weak pointer is invalid.");
    }
    auto* world_state = state->as<state::SimEnvWorldState>();
    auto* result_world_state = result->as<state::SimEnvWorldState>();
    auto* velocity_control = control->as<control::VelocityControl>();
    // lock the world for the whole propagation
//    _world->getLogger()->logDebug("Attempting to lock world", log_prefix);
    std::lock_guard<std::recursive_mutex> world_lock(_world->getMutex());
//    _world->getLogger()->logDebug("Locked world", log_prefix);
    // save current world state
    _world->saveState();
    // set the world to the start state
    state_space->setToState(_world, world_state);
    // now propagate for the duration of action
    float time = 0.0f;
    bool propagation_success = true;
    assert (_world->getPhysicsTimeStep() > 0.0f);
    Eigen::VectorXf vel(_controller->getTargetDimension());
//    logger->logDebug("Starting physics loop", log_prefix);
    while (time < velocity_control->getMaxDuration()) {
        // TODO we could do intermediate state checks here so we can abort prematurely in case of invalid collisions
        velocity_control->getVelocity(time, vel);
        _controller->setTargetVelocity(vel);
        _world->stepPhysics();
        time += _world->getPhysicsTimeStep();
    }
//    _world->getLogger()->logDebug("Physics loop terminated", log_prefix);

    // in case we have a semi dynamic propagation, continue propagating until rest
    if (_semi_dynamic) {
//        auto* semi_dyn_control = velocity_control->as<control::SemiDynamicVelocityControl>();
        auto* semi_dyn_control = dynamic_cast<control::SemiDynamicVelocityControl*>(velocity_control);
        assert(semi_dyn_control);
        float resting_time = 0.0f;
        vel.setZero();
        _controller->setTargetVelocity(vel); // constant zero target velocity
        // propagate until either t_max is reached, or the world is at rest
//        _world->getLogger()->logDebug("Semi-dynamic propagation enabled - starting 2nd physics loop", log_prefix);
        while (resting_time <= _t_max and not _world->atRest()) {
            _world->stepPhysics(1);
            resting_time += _world->getPhysicsTimeStep();
        }
        semi_dyn_control->setRestTime(resting_time);
        propagation_success = resting_time <= _t_max and _world->atRest();
//        _world->getLogger()->logDebug(boost::format("Semi-dynamic propagation terminated. Resting time is: %f")
//                                      % resting_time,
//                                      log_prefix);
    }

    state_space->extractState(_world, result_world_state);
    _world->restoreState();
    // TODO there are currently no validity checks performed here, i.e. is the outcoming state physicaly feasible
    // TODO (remember squeezing issue). Also no invalidation based on collisions.
    _world->getLogger()->logDebug(boost::format("Propagation finished. Success: %1%") % propagation_success,
                                  log_prefix);
    std::stringstream ss;
    result_world_state->print(ss);
    _world->getLogger()->logDebug("Resulting state: " + ss.str());
    return propagation_success;
}

bool SimEnvStatePropagator::canPropagateBackward() const {
    return false;
}
