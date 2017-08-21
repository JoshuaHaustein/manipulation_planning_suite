//
// Created by joshua on 8/21/17.
//

#include <mps/planner/ompl/control/SimEnvStatePropagator.h>
#include <mps/planner/ompl/state/SimEnvState.h>

using namespace mps::planner::ompl::control;
using namespace mps::planner::ompl::state;

SimEnvStatePropagator::SimEnvStatePropagator(::ompl::control::SpaceInformationPtr si, sim_env::WorldPtr world) :
        ::ompl::control::StatePropagator(si),
        _world(world)
{

}

SimEnvStatePropagator::~SimEnvStatePropagator() {
}

void SimEnvStatePropagator::propagate(const ::ompl::base::State* state, const ::ompl::control::Control* control,
                                      double duration, ::ompl::base::State* result) const {

}


bool SimEnvStatePropagator::propagate(const ::ompl::base::State* state, ::ompl::control::Control* control,
                                      ::ompl::base::State* result) const {

    return false;
}

bool SimEnvStatePropagator::canPropagateBackward() const {
    return false;
}
