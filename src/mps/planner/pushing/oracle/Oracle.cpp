//
// Created by joshua on 8/11/17.
//
#include <mps/planner/pushing/oracle/Oracle.h>

mps::planner::pushing::oracle::PushingOracle::~PushingOracle() = default;
mps::planner::pushing::oracle::RobotOracle::~RobotOracle() = default;

void mps::planner::pushing::oracle::PushingOracle::setObjectData(const std::vector<ObjectData>& object_data) {
    _object_data = object_data;
}