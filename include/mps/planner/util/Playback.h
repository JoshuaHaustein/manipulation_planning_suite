//
// Created by joshua on 9/6/17.
//

#ifndef MANIPULATION_PLANNING_SUITE_PLAYBACK_H
#define MANIPULATION_PLANNING_SUITE_PLAYBACK_H

#include <sim_env/SimEnv.h>
#include <sim_env/Controller.h>
#include <mps/planner/ompl/planning/Essentials.h>
#include <mps/planner/ompl/state/SimEnvState.h>

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
                 */
                void playPath(sim_env::WorldPtr world,
                              sim_env::RobotVelocityControllerPtr controller,
                              mps::planner::ompl::state::SimEnvWorldStateSpaceConstPtr state_space,
                              const mps::planner::ompl::planning::essentials::PathConstPtr& path);
            }
        }
    }
}

#endif //MANIPULATION_PLANNING_SUITE_PLAYBACK_H
