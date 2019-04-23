//
// Created by joshua on 8/14/17.
//

#include <iostream>
#include <mps/planner/ompl/control/Interfaces.h>

using namespace mps::planner::ompl::control;

SerializableControlSpace::~SerializableControlSpace() = default;

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

TimedControl::~TimedControl() = default;

PositionControl::~PositionControl() = default;
void PositionControl::getTarget(float dt, Eigen::VectorXf& vel) const
{
    getPosition(dt, vel);
}

VelocityControl::~VelocityControl() = default;
void VelocityControl::getTarget(float dt, Eigen::VectorXf& vel) const
{
    getVelocity(dt, vel);
}
SemiDynamicControl::~SemiDynamicControl() = default;
