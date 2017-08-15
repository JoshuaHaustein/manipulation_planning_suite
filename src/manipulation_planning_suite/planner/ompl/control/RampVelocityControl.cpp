//
// Created by joshua on 8/14/17.
//
#include <manipulation_planning_suite/planner/ompl/control/RampVelocityControl.h>

using namespace mps::planner::ompl::control;

RampVelocityControl::RampVelocityControl(const Eigen::VectorXf& max_accelarations) {
    _max_accelerations = max_accelarations;
}

RampVelocityControl::~RampVelocityControl() {
}

