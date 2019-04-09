//
// Created by joshua on 8/29/17.
//

#include <mps/planner/ompl/state/SimEnvState.h>
#include <mps/planner/ompl/state/goal/ObjectsRelocationGoal.h>
#include <mps/planner/util/Logging.h>
#include <mps/planner/util/Random.h>

#include <algorithm>
#include <limits>

using namespace mps::planner::ompl::state::goal;

RelocationGoalSpecification::RelocationGoalSpecification() = default;

RelocationGoalSpecification::RelocationGoalSpecification(const std::string& name,
    const Eigen::Vector3f& position,
    const Eigen::Quaternionf& orientation,
    float pos_tolerance,
    float angle_tolerance)
    : object_name(name)
    , goal_position(position)
    , goal_orientation(orientation)
    , position_tolerance(pos_tolerance)
    , orientation_tolerance(angle_tolerance)
{
}

RelocationGoalSpecification::RelocationGoalSpecification(const RelocationGoalSpecification& other) = default;

RelocationGoalSpecification& RelocationGoalSpecification::operator=(const RelocationGoalSpecification& other) = default;

ObjectsRelocationGoal::ObjectsRelocationGoal(::ompl::base::SpaceInformationPtr si,
    const RelocationGoalSpecification& goal_specification)
    : ObjectsRelocationGoal(si, std::vector<RelocationGoalSpecification>(1, goal_specification))
{
}

ObjectsRelocationGoal::ObjectsRelocationGoal(::ompl::base::SpaceInformationPtr si,
    const std::vector<RelocationGoalSpecification>& goal_specifications)
    : ::ompl::base::GoalSampleableRegion(si)
    , _goal_specifications(goal_specifications)
    , _max_num_attempts(100) // TODO make settable
{
    _state_sampler = si_->allocStateSampler();
    _rng = util::random::getDefaultRandomGenerator();
    ::ompl::base::StateSpacePtr ss = si_->getStateSpace();
    SimEnvWorldStateSpacePtr sim_env_ss = std::dynamic_pointer_cast<SimEnvWorldStateSpace>(ss);
    _validity_checker = si->getStateValidityChecker();
    if (!sim_env_ss) {
        throw std::logic_error("[mps::planner::ompl::state::goal::ObjectRelocationGoal] Unknown state space type encountered.");
    }

    _min_tolerance = std::numeric_limits<float>::infinity();
    for (auto& goal_specification : _goal_specifications) {
        int idx = sim_env_ss->getObjectIndex(goal_specification.object_name);
        if (idx == -1) {
            throw std::runtime_error("[mps::planner::ompl::state::goal::ObjectRelocationGoal] Could not retrieve state space for target object.");
        }
        goal_specification.id = (unsigned int)idx;
        _target_indices.push_back(goal_specification.id);
        _min_tolerance = std::min(goal_specification.position_tolerance, _min_tolerance);
    }
}

ObjectsRelocationGoal::~ObjectsRelocationGoal() = default;

void ObjectsRelocationGoal::sampleGoal(::ompl::base::State* state) const
{
    static const std::string log_prefix("[ObjectsRelocationGoal::sampleGoal]");
    // TODO do we need to ensure that the sampled state is valid??
    auto* sim_env_state = dynamic_cast<SimEnvWorldState*>(state);
    if (!sim_env_state) {
        throw std::logic_error("[mps::planner::ompl::state::goal::ObjectRelocationGoal::sampleGoal] Unknown state type encountered.");
    }
    unsigned int num_attempts = 0;
    bool has_valid_goal = false;

    while (num_attempts < _max_num_attempts and not has_valid_goal) {
        // first sample a normal state
        _state_sampler->sampleUniform(state);
        // now sample a pose of the target object around the target position
        // TODO this is hardcoded for sampling in a plane, either change documentation or make this more general
        #ifdef OMPL_NEW_VERSION
        std::vector<double> values(2);
        #else
        double values[2];
        #endif
        for (const auto& goal_specification : _goal_specifications) {
            #ifdef OMPL_NEW_VERSION
            _rng->uniformInBall(goal_specification.position_tolerance, values);
            #else
            _rng->uniformInBall(goal_specification.position_tolerance, 2, values);
            #endif
            Eigen::Vector3f position(goal_specification.goal_position);
            position[0] += values[0];
            position[1] += values[1];
            auto* object_state = sim_env_state->getObjectState(goal_specification.id);
            // TODO this is hardcoded for SE(2) objects
            // set the position of the target object
            Eigen::VectorXf config;
            object_state->getConfiguration(config);
            config[0] = position[0];
            config[1] = position[1];
            object_state->setConfiguration(config);
        }
        has_valid_goal = _validity_checker->isValid(state);
        ++num_attempts;
    }
    if (not has_valid_goal) {
        mps::planner::util::logging::logWarn("Failed to sample a valid goal, providing invalid one.", log_prefix);
    }
}

unsigned int ObjectsRelocationGoal::maxSampleCount() const
{
    return std::numeric_limits<unsigned int>::max();
}

double ObjectsRelocationGoal::distanceGoal(const ::ompl::base::State* st) const
{
    const auto* sim_env_state = dynamic_cast<const SimEnvWorldState*>(st);
    if (!sim_env_state) {
        throw std::logic_error("[mps::planner::ompl::state::goal::ObjectRelocationGoal::sampleGoal] Unknown state type encountered.");
    }
    double distance = 0.0;
    for (const auto& goal_specification : _goal_specifications) {
        auto object_state = sim_env_state->getObjectState(goal_specification.id);
        Eigen::VectorXf config = object_state->getConfiguration();
        // TODO this distance is also specific to SE(2)
        double dist_to_center = sqrt(pow(goal_specification.goal_position[0] - config[0], 2)
            + pow(goal_specification.goal_position[1] - config[1], 2));
        distance += std::max(dist_to_center - goal_specification.position_tolerance, 0.0);
    }
    return distance;
}

void ObjectsRelocationGoal::print(std::ostream& out) const
{
    out << "ObjectsRelocationGoal: targets: ";
    for (const auto& goal_specification : _goal_specifications) {
        out << goal_specification.object_name << ", goal position: " << goal_specification.goal_position.transpose() << ", ";
        out << "pos_tolerance: " << goal_specification.position_tolerance << std::endl;
    }
}

unsigned int ObjectsRelocationGoal::sampleTargetObjectIndex() const
{
    unsigned int idx = std::min(_rng->uniformInt(0, _target_indices.size()), ((int)_target_indices.size()) - 1);
    return _target_indices.at(idx);
}

float ObjectsRelocationGoal::getMinTolerance() const
{
    return _min_tolerance;
}
