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
    }
}

std::string mps::planner::util::yaml::algorithmTypeToString(mps::planner::pushing::PlanningProblem::AlgorithmType algo_type) {
    switch(algo_type) {
        case mps::planner::pushing::PlanningProblem::AlgorithmType::Naive:
            return "Naive";
        case mps::planner::pushing::PlanningProblem::AlgorithmType::OracleRRT:
            return "OracleRRT";
        case mps::planner::pushing::PlanningProblem::AlgorithmType::SliceOracleRRT:
            return "SliceOracleRRT";
        case mps::planner::pushing::PlanningProblem::AlgorithmType::CompleteSliceOracleRRT:
            return "CompleteSliceOracleRRT";
    }
}

mps::planner::pushing::PlanningProblem::OracleType mps::planner::util::yaml::stringToOracleType(const std::string& str) {
    if (str.compare("Human") == 0) {
        return mps::planner::pushing::PlanningProblem::OracleType::Human;
    } else if (str.compare("Learned") == 0) {
        return mps::planner::pushing::PlanningProblem::OracleType::Learned;
    } else {
        throw std::runtime_error("Unknown oracle type encountered: " + str);
    }
}

mps::planner::pushing::PlanningProblem::AlgorithmType mps::planner::util::yaml::stringToAlgorithmType(const std::string& str) {
    if (str.compare("Naive") == 0) {
        return mps::planner::pushing::PlanningProblem::AlgorithmType::Naive;
    } else if (str.compare("OracleRRT") == 0) {
        return mps::planner::pushing::PlanningProblem::AlgorithmType::OracleRRT;
    } else if (str.compare("SliceOracleRRT") == 0) {
        return mps::planner::pushing::PlanningProblem::AlgorithmType::SliceOracleRRT;
    } else if (str.compare("CompleteSliceOracleRRT") == 0) {
        return mps::planner::pushing::PlanningProblem::AlgorithmType::CompleteSliceOracleRRT;
    } else {
        throw std::runtime_error("Unknown algorithm type encountered: " + str);
    }
}
