//
// Created by joshua on 8/30/17.
//

#include <mps/planner/pushing/PushPlannerDistanceMeasure.h>
#include <boost/format.hpp>

using namespace mps::planner::pushing;

PushPlannerDistanceMeasure::PushPlannerDistanceMeasure(ompl::state::SimEnvWorldStateSpacePtr state_space,
                                                       std::vector<float> &weights):
    _weak_state_space(state_space),
    _weights(weights)
{
    _active_flags = std::vector<bool>(state_space->getNumObjects(), true);
    assert(_weights.size() == _active_flags.size());
}

PushPlannerDistanceMeasure::~PushPlannerDistanceMeasure() = default;

double PushPlannerDistanceMeasure::distance(const ::ompl::base::State *state1,
                                            const ::ompl::base::State *state2) const {
    auto state_space = getStateSpace();
    auto* sim_env_state_1 = state1->as<ompl::state::SimEnvWorldState>();
    auto* sim_env_state_2 = state2->as<ompl::state::SimEnvWorldState>();

    double dist = 0.0;
    for (unsigned int i = 0; i < state_space->getNumObjects(); ++i) {
        if (_active_flags.at(i)) {
            dist += _weights.at(i) * state_space->getSubspace(i)->distance(sim_env_state_1->getObjectState(i),
                                                                            sim_env_state_2->getObjectState(i));
        }
    }
    return dist;
}

void PushPlannerDistanceMeasure::setActive(unsigned int i, bool active) {
    _active_flags.at(i) = active;
}

void PushPlannerDistanceMeasure::setActive(const std::string &object_name, bool active) {
    auto state_space = getStateSpace();
    int idx = state_space->getObjectIndex(object_name);
    if (idx < 0) {
        throw std::runtime_error(boost::str(
                boost::format("[mps::planner::pushing::setActive] Unknown object %s ") % object_name));
    }
    _active_flags.at(idx) = active;
}

void PushPlannerDistanceMeasure::setAll(bool active) {
    for (unsigned int i = 0; i < _active_flags.size(); ++i) {
        _active_flags.at(i) = active;
    }
}

bool PushPlannerDistanceMeasure::isActive(unsigned int i) const {
    return _active_flags.at(i);
}

bool PushPlannerDistanceMeasure::isActive(const std::string& object_name) const {
    auto state_space = getStateSpace();
    int idx = state_space->getObjectIndex(object_name);
    if (idx < 0) {
        throw std::runtime_error(boost::str(
                boost::format("[mps::planner::pushing::setActive] Unknown object %s ") % object_name));
    }
    return _active_flags.at(idx);
}

mps::planner::ompl::state::SimEnvWorldStateSpacePtr PushPlannerDistanceMeasure::getStateSpace() const {
    auto pointer = _weak_state_space.lock();
    if (!pointer) {
        throw std::logic_error("[mps::planner::pushing::PushPlannerDistanceMeasure::getStateSpace] Can not access state space");
    }
    return pointer;
}
