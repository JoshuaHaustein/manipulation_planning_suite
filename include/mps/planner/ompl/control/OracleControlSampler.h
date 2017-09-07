//
// Created by joshua on 8/14/17.
//

#ifndef MANIPULATION_PLANNING_SUITE_ORACLECONTROLSAMPLER_H
#define MANIPULATION_PLANNING_SUITE_ORACLECONTROLSAMPLER_H

#include <ompl/control/DirectedControlSampler.h>
#include <mps/planner/ompl/control/Interfaces.h>
#include <mps/planner/pushing/oracle/Oracle.h>

namespace mps {
    namespace planner {
        namespace ompl {
            namespace control {
                class OracleControlSampler : public ::ompl::control::DirectedControlSampler {
                    // TODO implement oracle control sampler here; use oracle to sample a control
                    // TODO operate on RealValueParameterizedControl
                public:
                    OracleControlSampler(const ::ompl::control::SpaceInformation* si);
                    ~OracleControlSampler();
                    /**
                     *
                     * @param control
                     * @param source
                     * @param dest
                     * @return
                     */
                    unsigned int sampleTo(::ompl::control::Control* control,
                                          const ::ompl::base::State* source,
                                          ::ompl::base::State* dest) override;
                    /**
                     *
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
                };
            }
        }
    }
}
#endif //MANIPULATION_PLANNING_SUITE_ORACLECONTROLSAMPLER_H
