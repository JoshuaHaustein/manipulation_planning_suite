//
// Created by joshua on 8/14/17.
//
#include <mps/planner/ompl/control/OracleControlSampler.h>

using namespace mps::planner::ompl::control;
// TODO implement oracle control sampler

OracleControlSampler::OracleControlSampler(const ::ompl::control::SpaceInformation* si) :
    ::ompl::control::DirectedControlSampler(si)
{
   //TODO
}

OracleControlSampler::~OracleControlSampler() = default;

unsigned int OracleControlSampler::sampleTo(::ompl::control::Control *control, const ::ompl::base::State *source,
                                            ::ompl::base::State *dest) {
    // TODO
    return 0;
}

unsigned int OracleControlSampler::sampleTo(::ompl::control::Control *control, const ::ompl::control::Control *prev,
                                            const ::ompl::base::State *source, ::ompl::base::State *dest) {
    // TODO
    return 0;
}
