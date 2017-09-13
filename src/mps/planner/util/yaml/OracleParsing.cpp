//
// Created by joshua on 9/11/17.
//

#include <mps/planner/util/yaml/OracleParsing.h>

std::string mps::planner::util::yaml::oracleTypeToString(mps::planner::pushing::PlanningProblem::OracleType oracle_type) {
    switch(oracle_type) {
        case mps::planner::pushing::PlanningProblem::OracleType::Human:
            return "Human";
        case mps::planner::pushing::PlanningProblem::OracleType::Learned:
            return "Learned";
        case mps::planner::pushing::PlanningProblem::OracleType::None:
            return "None";
    }
}

mps::planner::pushing::PlanningProblem::OracleType mps::planner::util::yaml::stringToOracleType(const std::string& str) {
    if (str.compare("Human") == 0) {
        return mps::planner::pushing::PlanningProblem::OracleType::Human;
    } else if (str.compare("None") == 0) {
        return mps::planner::pushing::PlanningProblem::OracleType::None;
    } else {
        return mps::planner::pushing::PlanningProblem::OracleType::Learned;
    }
}

