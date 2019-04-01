//
// Created by joshua on 9/6/17.
//

#include <chrono>
#include <mps/planner/ompl/control/Interfaces.h>
#include <mps/planner/ompl/state/SimEnvState.h>
#include <mps/planner/pushing/algorithm/MultiExtendRRT.h>
#include <mps/planner/util/Logging.h>
#include <mps/planner/util/Playback.h>
#include <thread>

using namespace mps::planner::util::playback;
namespace mps_state = mps::planner::ompl::state;
namespace mps_control = mps::planner::ompl::control;
namespace mps_algorithm = mps::planner::pushing::algorithm;
namespace ob = ompl::base;
namespace oc = ompl::control;

void mps::planner::util::playback::playPath(sim_env::WorldPtr world,
    sim_env::RobotVelocityControllerPtr controller,
    mps::planner::ompl::state::SimEnvWorldStateSpaceConstPtr state_space,
    const mps::planner::ompl::planning::essentials::PathConstPtr& path,
    const std::function<bool()>& interrupt_callback,
    bool force_synch)
{
    const std::string log_prefix("[mps::planner::util::playback::playPath]");
    if (!path or path->getNumMotions() == 0) {
        logging::logWarn("The given path is empty or does not exist. Nothing to playback.",
            log_prefix);
        return;
    }
    auto* world_sim_env_state = dynamic_cast<mps_state::SimEnvWorldState*>(state_space->allocState());
    ::ompl::base::State const* wp_state = path->getConstMotion(0)->getConstState();
    //    auto* current_sim_env_state = wp_state->as<const mps_state::SimEnvWorldState>();
    auto* path_sim_env_state = dynamic_cast<mps_state::SimEnvWorldState const*>(wp_state);
    state_space->setToState(world, path_sim_env_state);
    unsigned int time_step_ms((unsigned int)(world->getPhysicsTimeStep() * 1000.0f));
    for (unsigned int idx = 0; idx < path->getNumMotions(); ++idx) {
        auto current_motion = path->getConstMotion(idx);
        auto push_motion = std::dynamic_pointer_cast<const mps_algorithm::PushMotion>(current_motion);
        path_sim_env_state = dynamic_cast<mps_state::SimEnvWorldState const*>(current_motion->getConstState());
        if (push_motion and push_motion->isTeleportTransit()) {
            // teleport robot to resulting state
            state_space->extractState(world, world_sim_env_state);
            auto robot_id = push_motion->getTargetId();
            // get robot state space
            auto robot_state_space = state_space->getSubspace(robot_id);
            robot_state_space->copyState(world_sim_env_state->getObjectState(robot_id), path_sim_env_state->getObjectState(robot_id));
            state_space->setToState(world, world_sim_env_state);
        } else {
            // simulate pushing action
            const ::ompl::control::Control* control = current_motion->getConstControl();
            auto* current_control = dynamic_cast<const mps_control::VelocityControl*>(control);
            float t = 0.0f;
            while (t < current_control->getMaxDuration()) {
                if (interrupt_callback())
                    return;
                controller->setTargetVelocity(current_control->getVelocity(t));
                world->stepPhysics(1);
                std::this_thread::sleep_for(std::chrono::milliseconds(time_step_ms));
                t += world->getPhysicsTimeStep();
            }
        }
        if (force_synch)
            state_space->setToState(world, path_sim_env_state);
    }
    state_space->freeState(world_sim_env_state);
}
