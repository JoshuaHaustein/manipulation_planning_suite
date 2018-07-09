//
// Created by joshua on 9/11/17.
//

#include <mps/planner/util/yaml/SortingParsing.h>

std::string mps::planner::util::yaml::valueFnTypeToString(mps::planner::sorting::PlanningProblem::ValueFunctionType value_fn_type) {
    switch(value_fn_type) {
        case mps::planner::sorting::PlanningProblem::ValueFunctionType::Entropy:
            return "Entropy";
        case mps::planner::sorting::PlanningProblem::ValueFunctionType::Learned:
            return "Learned";
    }
    return "UNDEFINED";
}

std::string mps::planner::util::yaml::algorithmTypeToString(mps::planner::sorting::PlanningProblem::AlgorithmType algo_type) {
    switch(algo_type) {
        case mps::planner::sorting::PlanningProblem::AlgorithmType::DeterministicMCTS:
            return "DeterministicMCTS";
        case mps::planner::pushing::PlanningProblem::AlgorithmType::NonDeterministicMCTS:
            return "NonDeterministicMCTS";
    }
    return "UNDEFINED";
}

mps::planner::sorting::PlanningProblem::ValueFunctionType mps::planner::util::yaml::stringToValueFnType(const std::string& str) {
    if (str.compare("Entropy") == 0) {
        return mps::planner::sorting::PlanningProblem::ValueFunctionType::Entropy;
    } else if (str.compare("Learned") == 0) {
        return mps::planner::sorting::PlanningProblem::ValueFunctionType::Learned;
    } else {
        throw std::runtime_error("Unknown oracle type encountered: " + str);
    }
}

mps::planner::sorting::PlanningProblem::AlgorithmType mps::planner::util::yaml::stringToSortingAlgorithmType(const std::string& str) {
    if (str.compare("DeterministicMCTS") == 0) {
        return mps::planner::sorting::PlanningProblem::AlgorithmType::DeterministicMCTS;
    } else if (str.compare("NonDeterministicMCTS") == 0) {
        return mps::planner::sorting::PlanningProblem::AlgorithmType::NonDetereministicMCTS;
    } else {
        throw std::runtime_error("Unknown algorithm type encountered: " + str);
    }
}

void mps::planner::util::yaml::configurePlanningProblem(mps::planner::sorting::PlanningProblem &problem,
                                                        const SortingPlanningProblemDesc &problem_desc) {
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
    problem.planning_time_out = problem_desc.planning_timeout;
    problem.t_max = problem_desc.t_max;
    problem.num_control_samples = problem_desc.num_control_samples;
    problem.algorithm_type = problem_desc.algorithm_type;
    problem.value_fn_type = problem_desc.value_fn_type;
    problem.sorting_groups = problem_desc.sorting_groups;
    // Collisions policy
    problem.collision_policy.setStaticCollisions(problem_desc.collision_policy.static_collisions_allowed);
    for (auto& forbidden_col : problem_desc.collision_policy.static_collisions_blacklist) {
        problem.collision_policy.setStaticCollisions(forbidden_col, false);
    }
    for (auto& forbidden_col : problem_desc.collision_policy.collisions_blacklist) {
        problem.collision_policy.setCollision(forbidden_col.first, forbidden_col.second, false);
    }
}
