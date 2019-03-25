//
// Created by joshua on 8/30/17.
//

#include <boost/format.hpp>
#include <mps/planner/ompl/state/SimEnvWorldStateDistanceMeasure.h>
#include <mps/planner/util/Logging.h>

using namespace mps::planner::ompl::state;
namespace mps_logging = mps::planner::util::logging;

SimEnvWorldStateDistanceMeasure::SimEnvWorldStateDistanceMeasure(ompl::state::SimEnvWorldStateSpacePtr state_space,
    const std::vector<float>& weights)
    : _weak_state_space(state_space)
    , _weights(weights)
{
    _active_flags = std::vector<bool>(state_space->getNumObjects(), true);
    if (_weights.size() == 0) {
        _weights.resize(_active_flags.size(), 1.0f);
    }
}

SimEnvWorldStateDistanceMeasure::~SimEnvWorldStateDistanceMeasure() = default;

void SimEnvWorldStateDistanceMeasure::setWeights(const std::vector<float>& weights)
{
    if (weights.size() == _weights.size()) {
        _weights = weights;
    } else {
        mps::planner::util::logging::logErr("Insufficient number of weights provided. Not resetting weights.",
            "[mps::planner::pushing::SimEnvWorldStateDistanceMeasure::setWeights]");
    }
}

double SimEnvWorldStateDistanceMeasure::distance(const ::ompl::base::State* state1,
    const ::ompl::base::State* state2) const
{
    auto state_space = getStateSpace();
    auto* sim_env_state_1 = state1->as<ompl::state::SimEnvWorldState>();
    auto* sim_env_state_2 = state2->as<ompl::state::SimEnvWorldState>();

    double dist = 0.0;
    for (unsigned int i = 0; i < state_space->getNumObjects(); ++i) {
        if (_active_flags.at(i)) {
            dist += _weights.at(i) * state_space->getSubspace(i)->distance(sim_env_state_1->getObjectState(i), sim_env_state_2->getObjectState(i));
        }
    }
    //    mps_logging::logDebug("Computed distance between state " + sim_env_state_1->toString() +
    //                                  " and state " + sim_env_state_2->toString() + " distance is: " + std::to_string(dist),
    //                          "[SimEnvWorldStateDistanceMeasure]");
    return dist;
}

void SimEnvWorldStateDistanceMeasure::setActive(unsigned int i, bool active)
{
    _active_flags.at(i) = active;
}

void SimEnvWorldStateDistanceMeasure::setActive(const std::string& object_name, bool active)
{
    auto state_space = getStateSpace();
    int idx = state_space->getObjectIndex(object_name);
    if (idx < 0) {
        throw std::runtime_error(boost::str(
            boost::format("[mps::planner::pushing::setActive] Unknown object %s ") % object_name));
    }
    _active_flags.at(idx) = active;
}

void SimEnvWorldStateDistanceMeasure::setAll(bool active)
{
    for (unsigned int i = 0; i < _active_flags.size(); ++i) {
        _active_flags.at(i) = active;
    }
}

bool SimEnvWorldStateDistanceMeasure::isActive(unsigned int i) const
{
    return _active_flags.at(i);
}

bool SimEnvWorldStateDistanceMeasure::isActive(const std::string& object_name) const
{
    auto state_space = getStateSpace();
    int idx = state_space->getObjectIndex(object_name);
    if (idx < 0) {
        throw std::runtime_error(boost::str(
            boost::format("[mps::planner::pushing::setActive] Unknown object %s ") % object_name));
    }
    return _active_flags.at(idx);
}

mps::planner::ompl::state::SimEnvWorldStateSpacePtr SimEnvWorldStateDistanceMeasure::getStateSpace() const
{
    auto pointer = _weak_state_space.lock();
    if (!pointer) {
        throw std::logic_error("[mps::planner::pushing::SimEnvWorldStateDistanceMeasure::getStateSpace] Can not access state space");
    }
    return pointer;
}

/*********************************************************************************************************************/
/**************************************** SimEnvWorldStateDistanceProjection *****************************************/
/*********************************************************************************************************************/
SimEnvWorldStateDistanceProjection::SimEnvWorldStateDistanceProjection(ompl::state::SimEnvWorldStateSpacePtr state_space,
    unsigned int obj_id)
    : _weak_world_state_space(state_space)
{
    setObjectId(obj_id);
}

SimEnvWorldStateDistanceProjection::~SimEnvWorldStateDistanceProjection() = default;

void SimEnvWorldStateDistanceProjection::setObjectId(unsigned int id)
{
    static const std::string log_prefix("[mps::planner::ompl::state::SimEnvWorldStateDistanceProjection::setObjectId]");
    auto world_state_space = _weak_world_state_space.lock();
    if (!world_state_space) {
        throw std::logic_error(log_prefix + " Weak pointer expired: could not access world state space.");
    }
    auto sub_state_space = std::dynamic_pointer_cast<ompl::state::SimEnvObjectStateSpace>(world_state_space->getSubspace(id));
    if (!sub_state_space) {
        throw std::logic_error(log_prefix + " Could not retrieve state space of object " + std::to_string(id));
    }
    _weak_state_space = sub_state_space;
    _obj_id = id;
}

double SimEnvWorldStateDistanceProjection::distance(const ::ompl::base::State* state1, const ::ompl::base::State* state2) const
{
    auto state_space = _weak_state_space.lock();
    if (!state_space) {
        throw std::logic_error("[mps::planner::pushing::SimEnvWorldStateDistanceProjection::distance] Can not access state space");
    }
    auto* sim_env_state_1 = state1->as<ompl::state::SimEnvWorldState>();
    auto* sim_env_state_2 = state2->as<ompl::state::SimEnvWorldState>();

    return state_space->distance(sim_env_state_1->getObjectState(_obj_id), sim_env_state_2->getObjectState(_obj_id));
}
