//
// Created by joshua on 9/6/17.
//

#ifndef MANIPULATION_PLANNING_SUITE_PLAYBACK_H
#define MANIPULATION_PLANNING_SUITE_PLAYBACK_H

#include <mps/planner/ompl/planning/Essentials.h>
#include <mps/planner/ompl/state/SimEnvState.h>
#include <sim_env/Controller.h>
#include <sim_env/SimEnv.h>

namespace mps {
namespace planner {
    namespace util {
        namespace playback {
            /**
                 * Forward simulates the given path in realtime.
                 * This function blocks the calling thread until the execution is finished.
                 * This function is intended to be used for playback, i.e. visualization of a path.
                 * @param world - the world to execute the path in
                 * @param robot - the robot that is executing the control actions stored in path
                 * @param state space - State space of the states in path. Required to interpret states.
                 * @param path - a path consisting of motions that store tuples (SimEnvWorldState, VelocityControl)
                 * @param interrupt_callback - a function that returns true if the playback should be interrupted
                 * @param bool  force_synch - if true, synchronizes the world state with the state in the path after each action
                 */
            void playPath(sim_env::WorldPtr world,
                sim_env::RobotVelocityControllerPtr controller,
                mps::planner::ompl::state::SimEnvWorldStateSpaceConstPtr state_space,
                const mps::planner::ompl::planning::essentials::PathConstPtr& path,
                const std::function<bool()>& interrupt_callback,
                bool force_synch);

            /**
                 * Forward simulates the given motion in realtime.
                 * This function blocks the calling thread until the execution of the motion is finished.
                 * It is intended to be used to simulate execution, i.e. to test an execution monitor.
                 * @param world - the world to execute actions in
                 * @param controller - robot controller in that world
                 * @param state_space - world state space
                 * @param interrupt_callback - a function that returns true if the playback should be interrupted
                 * @param motion - the motion to execute
                 * @param result_state - after execution contains the resulting state
                 * @return false, if any error occurred, else true
                 */
            bool playMotion(sim_env::WorldPtr world,
                sim_env::RobotVelocityControllerPtr controller,
                mps::planner::ompl::state::SimEnvWorldStateSpaceConstPtr state_space,
                const std::function<bool()>& interrupt_callback,
                const mps::planner::ompl::planning::essentials::MotionConstPtr& motion,
                mps::planner::ompl::state::SimEnvWorldState* result_state);
        }
    }
}
}

#endif //MANIPULATION_PLANNING_SUITE_PLAYBACK_H
