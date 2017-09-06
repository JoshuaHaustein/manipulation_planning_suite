//
// Created by joshua on 9/6/17.
//

#include <mps/planner/util/Playback.h>
#include <mps/planner/util/Logging.h>
#include <mps/planner/ompl/state/SimEnvState.h>
#include <mps/planner/ompl/control/Interfaces.h>
#include <thread>
#include <chrono>

using namespace mps::planner::util::playback;
namespace mps_state = mps::planner::ompl::state;
namespace mps_control = mps::planner::ompl::control;
namespace ob = ompl::base;
namespace oc = ompl::control;

void mps::planner::util::playback::playPath(sim_env::WorldPtr world,
                                            sim_env::RobotVelocityControllerPtr controller,
                                            mps::planner::ompl::state::SimEnvWorldStateSpaceConstPtr state_space,
                                            const mps::planner::ompl::planning::essentials::PathConstPtr &path) {
    const std::string log_prefix("[mps::planner::util::playback::playPath]");
    if (!path or path->getNumMotions() <= 1) {
        logging::logWarn("The given path is non-existant, empty or trivial, i.e. only a single state. Nothing to playback.",
                         log_prefix);
        return;
    }
    ::ompl::base::State const * current_state = path->getConstMotion(0)->getConstState();
//    auto* current_sim_env_state = current_state->as<const mps_state::SimEnvWorldState>();
    auto* current_sim_env_state = dynamic_cast<mps_state::SimEnvWorldState const*>(current_state);
    state_space->setToState(world, current_sim_env_state);
    unsigned int time_step_ms((unsigned int)(world->getPhysicsTimeStep() * 1000.0f));
    for (unsigned int idx = 0; idx < path->getNumMotions(); ++idx) {
        const ::ompl::control::Control* control = path->getConstMotion(idx)->getConstControl();
        auto* current_control = dynamic_cast<const mps_control::VelocityControl*>(control);
        float time = 0.0f;
        while (time < current_control->getMaxDuration()) {
            controller->setTargetVelocity(current_control->getVelocity(time));
            world->stepPhysics(1);
            std::this_thread::sleep_for(std::chrono::milliseconds(time_step_ms));
            time += world->getPhysicsTimeStep();
        }
    }
}

