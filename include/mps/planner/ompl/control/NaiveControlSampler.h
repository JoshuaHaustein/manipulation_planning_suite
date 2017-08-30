//
// Created by joshua on 8/14/17.
//

#ifndef MANIPULATION_PLANNING_SUITE_NAIVECONTROLSAMPLER_H
#define MANIPULATION_PLANNING_SUITE_NAIVECONTROLSAMPLER_H

#include <ompl/control/SpaceInformation.h>
#include <ompl/control/DirectedControlSampler.h>
#include <ompl/control/Control.h>

namespace mps {
    namespace planner {
        namespace ompl {
            namespace control {
                /**
                 * A nairve control sampler similar to SimpleDirectedControlSampler which can be used in combination
                 * with a SimEnvStatePropagator, i.e. it allows semi-dynamic control sampling.
                 * The sampler samples k controls, propagates each of them and selects the one that reached
                 * the state closest to the target state.
                 */
                class NaiveControlSampler : public ::ompl::control::DirectedControlSampler {
                public:
                    /**
                     * Creates a new NaiveControlSampler.
                     * The state space stored in si must be an instance of SimEnvWorldStateSpace.
                     * The state propagator in si must be an instance of SimEnvStatePropagator.
                     * The control space stored in si must be compatible with SimEnvStatePropagator.
                     * @param si - space information
                     * @param k - number of control samples to draw each time a sample is requested
                     */
                    NaiveControlSampler(const ::ompl::control::SpaceInformation* si, unsigned int k=10);
                    ~NaiveControlSampler();

                    /**
                     * Samples a control approximately moving the system from state source towards dest.
                     * @param control - the output control
                     * @param source - the start state (must be an instance of SimEnvWorldState)
                     * @param dest - the destination state
                     * @return 1
                     */
                    unsigned int sampleTo(::ompl::control::Control* control,
                                          const ::ompl::base::State* source,
                                          ::ompl::base::State* dest) override;
                    /**
                     * Same as other sampleTo method.
                     * @param control
                     * @param prev
                     * @param source
                     * @param dest
                     * @return
                     */
                    unsigned int sampleTo(::ompl::control::Control* control,
                                          const ::ompl::control::Control* prev,
                                          const ::ompl::base::State* source,
                                          ::ompl::base::State* dest) override;

                    void setK(unsigned int k);
                    unsigned int getK() const;
                private:
                    ::ompl::control::ControlSamplerPtr _control_sampler;
                    ::ompl::control::Control* _best_control;
                    ::ompl::base::State* _result_state;
                    unsigned int _k;
                };
            }
        }
    }
}
#endif //MANIPULATION_PLANNING_SUITE_NAIVECONTROLSAMPLER_H
