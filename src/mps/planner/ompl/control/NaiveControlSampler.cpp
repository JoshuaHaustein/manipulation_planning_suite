//
// Created by joshua on 8/30/17.
//

#include <mps/planner/ompl/control/NaiveControlSampler.h>
#include <mps/planner/ompl/control/SimEnvStatePropagator.h>
#include <mps/planner/util/Logging.h>
#include <boost/format.hpp>

using namespace mps::planner::ompl::control;
namespace logging = mps::planner::util::logging;

NaiveControlSampler::NaiveControlSampler(const ::ompl::control::SpaceInformation* si, unsigned int k) :
    ::ompl::control::DirectedControlSampler(si), _k(k)
{
    _control_sampler = si->allocControlSampler();
    _best_control = si->allocControl();
    _best_state = si->allocState();
    _result_state = si->allocState();
    ::ompl::control::StatePropagatorPtr propagator = si_->getStatePropagator();
    _state_propagator = std::dynamic_pointer_cast<SimEnvStatePropagator>(propagator);
    if (!_state_propagator) {
        std::runtime_error("[mps::planner::ompl::control::NaiveControlSampler::sampleTo]"
                                   " Could not retrieve valid SimEnvStatePropagator.");
    }
}

NaiveControlSampler::~NaiveControlSampler() {
    si_->freeState(_result_state);
    si_->freeState(_best_state);
    si_->freeControl(_best_control);
}

unsigned int NaiveControlSampler::sampleTo(::ompl::control::Control *control,
                                           const ::ompl::base::State *source,
                                           ::ompl::base::State *dest) {
    static const std::string prefix("[mps::planner::ompl::control::NaiveControlSampler::sampleTo]");
    logging::logDebug("Sampling control towards a state", prefix);
    // sample k controls and return the best
    // initial guess is that it's best to do nothing
    double best_distance = si_->distance(source, dest);
    unsigned int num_steps = 0;
    si_->nullControl(_best_control);
    si_->copyState(_best_state, source);
    // let's check k controls
    logging::logDebug(boost::str(boost::format("Sampling %i controls") % _k), prefix);
    for (unsigned int i = 0; i < _k; ++i) {
        // sample a control given the source state
        _control_sampler->sample(control, source);
        // propagate its result
        bool propagation_success = _state_propagator->propagate(source, control, _result_state);
        if (propagation_success and si_->isValid(_result_state)) { // if we have a valid outcome
            // compute the distance of the resulting state
            double new_distance = si_->distance(_result_state, dest);
            // if it is better than our previous one, copy the control
            if (new_distance < best_distance) {
                si_->copyControl(_best_control, control);
                si_->copyState(_best_state, _result_state);
                best_distance = new_distance;
                num_steps = 1;
            }
            logging::logDebug(boost::str(boost::format("Control %i succeeded, resulting distance is %f, best distance is %f")
                                     % i % new_distance % best_distance), prefix);
        }
    }
    logging::logDebug(boost::str(boost::format("Finished control sampling. Best distance is %f")
                             % best_distance), prefix);
    // finally ensure that control points to a control with the best values
    si_->copyControl(control, _best_control);
    si_->copyState(dest, _best_state);
    return num_steps;
}

unsigned int NaiveControlSampler::sampleTo(::ompl::control::Control* control,
                                          const ::ompl::control::Control* prev,
                                          const ::ompl::base::State* source,
                                          ::ompl::base::State* dest) {
   return sampleTo(control, source, dest);
}

void NaiveControlSampler::setK(unsigned int k) {
   if (k > 0) {
      _k = k;
   } else {
      // TODO Warning
       _k = 1;
   }
}

unsigned int NaiveControlSampler::getK() const {
   return _k;
}
