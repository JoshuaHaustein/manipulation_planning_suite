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
    sim_env::RobotControllerPtr controller,
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
    for (unsigned int idx = 0; idx < path->getNumMotions(); ++idx) {
        if (interrupt_callback()) {
            break;
        }
        auto current_motion = path->getConstMotion(idx);
        playMotion(world, controller, state_space, interrupt_callback, current_motion, world_sim_env_state);
        if (force_synch) {
            path_sim_env_state = dynamic_cast<mps_state::SimEnvWorldState const*>(current_motion->getConstState());
            state_space->setToState(world, path_sim_env_state);
        }
    }
    state_space->freeState(world_sim_env_state);
}

bool mps::planner::util::playback::playMotion(sim_env::WorldPtr world,
    sim_env::RobotControllerPtr controller,
    mps::planner::ompl::state::SimEnvWorldStateSpaceConstPtr state_space,
    const std::function<bool()>& interrupt_callback,
    const mps::planner::ompl::planning::essentials::MotionConstPtr& motion,
    mps::planner::ompl::state::SimEnvWorldState* result_state)
{
    const std::string log_prefix("[mps::planner::util::playback::playMotion]");
    auto* target_state = dynamic_cast<mps_state::SimEnvWorldState const*>(motion->getConstState());
    unsigned int time_step_ms((unsigned int)(world->getPhysicsTimeStep() * 1000.0f));
    auto push_motion = std::dynamic_pointer_cast<const mps_algorithm::PushMotion>(motion);
    if (push_motion and push_motion->isTeleportTransit()) {
        // teleport robot to target state
        state_space->extractState(world, result_state);
        auto robot_id = push_motion->getTargetId();
        // get robot state space
        auto robot_state_space = state_space->getSubspace(robot_id);
        robot_state_space->copyState(result_state->getObjectState(robot_id), target_state->getObjectState(robot_id));
        state_space->setToState(world, result_state);
    } else {
        // simulate pushing action
        const ::ompl::control::Control* control = motion->getConstControl();
        auto* timed_control = dynamic_cast<const mps_control::TimedControl*>(control);
        float t = 0.0f;
        Eigen::VectorXf target;
        while (t < timed_control->getDuration()) {
            if (interrupt_callback())
                return false;
            timed_control->getTarget(t, target);
            controller->setTarget(target);
            world->stepPhysics(1);
            std::this_thread::sleep_for(std::chrono::milliseconds(time_step_ms));
            t += world->getPhysicsTimeStep();
        }
        // read resulting state
        state_space->extractState(world, result_state);
    }
    return true;
}
