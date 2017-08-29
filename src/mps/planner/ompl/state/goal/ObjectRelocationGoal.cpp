//
// Created by joshua on 8/29/17.
//

#include <mps/planner/ompl/state/goal/ObjectRelocationGoal.h>
#include <mps/planner/ompl/state/SimEnvState.h>
#include <mps/planner/util/Random.h>

#include <limits>

using namespace mps::planner::ompl::state::goal;

ObjectRelocationGoal::ObjectRelocationGoal(::ompl::base::SpaceInformationPtr si, const std::string &target_obj,
                                           const Eigen::Vector3f &goal_position,
                                           const Eigen::Quaternionf &goal_orientation, float position_tolerance,
                                           float orientation_tolerance) :
    ::ompl::base::GoalSampleableRegion(si),
    _target_name(target_obj),
    _goal_position(goal_position),
    _goal_orientation(goal_orientation),
    _position_tolerance(position_tolerance),
    _orientation_tolerance(orientation_tolerance)
{
    _state_sampler = si_->allocStateSampler();
    _rng = util::random::getDefaultRandomGenerator();
    ::ompl::base::StateSpacePtr ss = si_->getStateSpace();
    SimEnvWorldStateSpacePtr sim_env_ss = std::dynamic_pointer_cast<SimEnvWorldStateSpace>(ss);
    if(!sim_env_ss) {
        throw std::logic_error("[mps::planner::ompl::state::goal::ObjectRelocationGoal] Unknown state space type encountered.");
    }
    int idx = sim_env_ss->getObjectIndex(_target_name);
    if(idx == -1) {
        throw std::runtime_error("[mps::planner::ompl::state::goal::ObjectRelocationGoal] Could not retrieve state space for target object.");
    }
    _target_id = (unsigned int) idx;
}

ObjectRelocationGoal::~ObjectRelocationGoal() = default;

void ObjectRelocationGoal::sampleGoal(::ompl::base::State *state) const {
    auto* sim_env_state = dynamic_cast<SimEnvWorldState*>(state);
    if (!sim_env_state) {
        throw std::logic_error("[mps::planner::ompl::state::goal::ObjectRelocationGoal::sampleGoal] Unknown state type encountered.");
    }
    // first sample a normal state
    _state_sampler->sampleUniform(state);
    // now sample a pose of the target object around the target position
    // TODO this is hardcoded for sampling in a plane, either change documentation or make this more general
    double values[2];
    _rng->uniformInBall(_position_tolerance, 2, values);
    Eigen::Vector3f position(_goal_position);
    position[0] += values[0];
    position[1] += values[1];
    auto* object_state = sim_env_state->getObjectState(_target_id);
    // TODO this is hardcoded for SE(2) objects
    // set the position of the target object
    Eigen::VectorXf config;
    object_state->getConfiguration(config);
    config[0] = position[0];
    config[1] = position[1];
    object_state->setConfiguration(config);
}

unsigned int ObjectRelocationGoal::maxSampleCount() const {
    return std::numeric_limits<unsigned int>::max();
}

double ObjectRelocationGoal::distanceGoal(const ::ompl::base::State *st) const {
    auto* sim_env_state = dynamic_cast<const SimEnvWorldState*>(st);
    if (!sim_env_state) {
        throw std::logic_error("[mps::planner::ompl::state::goal::ObjectRelocationGoal::sampleGoal] Unknown state type encountered.");
    }
    auto object_state = sim_env_state->getObjectState(_target_id);
    Eigen::VectorXf config = object_state->getConfiguration();
    // TODO this distance is also specific to SE(2)
    return sqrt(pow(_goal_position[0] - config[0], 2) + pow(_goal_position[1] - config[1], 2));
}

void ObjectRelocationGoal::print(std::ostream& out) const {
    out << "ObjectRelocationGoal: target: " << _target_name << ", goal position: " << _goal_position.transpose()
        << ", pos_tolerance: " << _position_tolerance << std::endl;
}
