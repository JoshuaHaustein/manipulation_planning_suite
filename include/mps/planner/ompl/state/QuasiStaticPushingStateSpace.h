#pragma once

#include <ompl/base/spaces/DubinsStateSpace.h>

namespace mps {
namespace planner {
    namespace ompl {
        namespace state {
            // Essentially just a DubinsStateSpace, but you can modify the turning radius after creation.
            class QuasiStaticPushingStateSpace : public ::ompl::base::DubinsStateSpace {
            public:
                QuasiStaticPushingStateSpace()
                    : ::ompl::base::DubinsStateSpace(1.0, false)
                {
                }
                ~QuasiStaticPushingStateSpace() = default;
                void setTurningRadius(double rad)
                {
                    rho_ = rad;
                }

                float getTurningRadius() const
                {
                    return rho_;
                }
            };
        }
    }
}
}