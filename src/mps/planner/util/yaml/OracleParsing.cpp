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
    return "UNDEFINED";
}

std::string mps::planner::util::yaml::localPlannerTypeToString(mps::planner::pushing::PlanningProblem::LocalPlanner planner_type) {
    switch(planner_type) {
        case mps::planner::pushing::PlanningProblem::LocalPlanner::Line:
            return "Line";
        case mps::planner::pushing::PlanningProblem::LocalPlanner::ElasticBand:
            return "ElasticBand";
    }
    return "UNDEFINED";
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
        case mps::planner::pushing::PlanningProblem::AlgorithmType::GNATSamplingSliceOracleRRT:
            return "GNATSamplingSliceOracleRRT";
        case mps::planner::pushing::PlanningProblem::AlgorithmType::SemanticGNATSamplingSliceOracleRRT:
            return "SemanticGNATSamplingSliceOracleRRT";
    }
    return "UNDEFINED";
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
    } else if (str.compare("GNATSamplingSliceOracleRRT") == 0) {
        return  mps::planner::pushing::PlanningProblem::AlgorithmType::GNATSamplingSliceOracleRRT;
    } else if (str.compare("SemanticGNATSamplingSliceOracleRRT") == 0) {
        return  mps::planner::pushing::PlanningProblem::AlgorithmType::SemanticGNATSamplingSliceOracleRRT;
    } else {
        throw std::runtime_error("Unknown algorithm type encountered: " + str);
    }
}

mps::planner::pushing::PlanningProblem::LocalPlanner mps::planner::util::yaml::stringToLocalPlannerType(const std::string& str) {
    if (str.compare("Line") == 0) {
        return mps::planner::pushing::PlanningProblem::LocalPlanner::Line;
    } else if (str.compare("ElasticBand") == 0) {
        return mps::planner::pushing::PlanningProblem::LocalPlanner::ElasticBand;
    } else {
        throw std::runtime_error("Unknown local planner type encountered: " + str);
    }
}

void mps::planner::util::yaml::configurePlanningProblem(mps::planner::pushing::PlanningProblem &problem,
                                                        const OraclePlanningProblemDesc &problem_desc) {
    // load control limits
    problem.control_limits.velocity_limits = problem_desc.control_limits.velocity_limits;
    problem.control_limits.duration_limits = problem_desc.control_limits.duration_limits;
    problem.control_limits.acceleration_limits = problem_desc.control_limits.acceleration_limits;
    // and subspaces
    for (auto& indices : problem_desc.control_limits.subspaces) {
        Eigen::Array2f limits;
        limits[0] = 0.0;
        limits[1] = problem_desc.control_limits.velocity_limits(indices[0]);
        problem.control_subspaces.emplace_back(ompl::control::RampVelocityControlSpace::ControlSubspace(indices, limits));
    }
    // Workspace bounds
    problem.workspace_bounds.x_limits = problem_desc.x_limits;
    problem.workspace_bounds.y_limits = problem_desc.y_limits;
    problem.workspace_bounds.z_limits = problem_desc.z_limits;
    problem.workspace_bounds.max_rotation_vel = problem_desc.max_rotation_vel;
    problem.workspace_bounds.max_velocity = problem_desc.max_velocity;
    // Various planner parameters
    problem.t_max = problem_desc.t_max;
    problem.planning_time_out = problem_desc.planning_timeout;
    problem.num_control_samples = problem_desc.num_control_samples;
    problem.algorithm_type = problem_desc.algorithm_type;
    problem.oracle_type = problem_desc.oracle_type;
    problem.local_planner_type = problem_desc.local_planner_type;
    problem.goal_bias = problem_desc.goal_bias;
    problem.robot_bias = problem_desc.robot_bias;
    problem.target_bias = problem_desc.target_bias;
    // TODO b_semi_dynamic
    // Collisions policy
    problem.collision_policy.setStaticCollisions(problem_desc.collision_policy.static_collisions_allowed);
    for (auto& forbidden_col : problem_desc.collision_policy.static_collisions_blacklist) {
        problem.collision_policy.setStaticCollisions(forbidden_col, false);
    }
    for (auto& forbidden_col : problem_desc.collision_policy.collisions_blacklist) {
        problem.collision_policy.setCollision(forbidden_col.first, forbidden_col.second, false);
    }
}
