//
// Created by joshua on 8/14/17.
//

#include <iostream>
#include <mps/planner/ompl/control/Interfaces.h>

using namespace mps::planner::ompl::control;

RealValueParameterizedControl::~RealValueParameterizedControl() = default;

void RealValueParameterizedControl::serializeInNumbers(std::ostream& ostream) const
{
    Eigen::VectorXf params;
    getParameters(params);
    ostream << params.transpose().format(eigen_format);
}

void RealValueParameterizedControl::deserializeFromNumbers(std::istream& istream)
{
    Eigen::VectorXf params;
    getParameters(params);
    for (unsigned int i = 0; i < params.size(); ++i) {
        std::string next_value;
        std::getline(istream, next_value, ',');
        params[i] = std::stof(next_value);
    }
    setParameters(params);
}

unsigned int RealValueParameterizedControl::getNumNumbers() const
{
    return getNumParameters();
}

VelocityControl::~VelocityControl() = default;
SemiDynamicControl::~SemiDynamicControl() = default;
