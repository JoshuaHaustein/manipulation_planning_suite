//
// Created by joshua on 8/14/17.
//

#include <mps/planner/ompl/control/Interfaces.h>

using namespace mps::planner::ompl::control;

RealValueParameterizedControl::~RealValueParameterizedControl() = default;

void RealValueParameterizedControl::serializeInNumbers(std::ostream &ostream) const {
    Eigen::VectorXf params;
    getParameters(params);
    ostream << params.transpose().format(eigen_format);
}

VelocityControl::~VelocityControl() = default;
SemiDynamicVelocityControl::~SemiDynamicVelocityControl() = default;

