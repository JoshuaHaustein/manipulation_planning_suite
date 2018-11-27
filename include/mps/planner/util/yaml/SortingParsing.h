//
// Created by joshua on 7/09/18.
//

#ifndef MANIPULATION_PLANNING_SUITE_SORTPARSING_H
#define MANIPULATION_PLANNING_SUITE_SORTPARSING_H

#include <mps/planner/sorting/PushSortingPlanner.h>
#include <mps/planner/util/yaml/OracleParsing.h>
#include <sim_env/utils/YamlUtils.h>
#include <yaml-cpp/yaml.h>
#include <Eigen/Core>
#include <vector>

namespace mps {
    namespace planner {
        namespace util {
            namespace yaml {

                struct SortingPlanningProblemDesc {
                    std::string world_file;
                    Eigen::Array2f x_limits;
                    Eigen::Array2f y_limits;
                    Eigen::Array2f z_limits;
                    float max_velocity;
                    float max_rotation_vel;
                    float planning_timeout;
                    ControlLimitsDesc control_limits;
                    CollisionPolicyDesc collision_policy;
                    float t_max;
                    mps::planner::sorting::PlanningProblem::AlgorithmType algorithm_type;
                    mps::planner::sorting::PlanningProblem::ValueFunctionType value_fn_type;
                    std::map<std::string, unsigned int> sorting_groups;
                    // TODO optionally add parameters here
                    unsigned int num_control_samples;
                };

                std::string valueFnTypeToString(mps::planner::sorting::PlanningProblem::ValueFunctionType value_fn_type);
                std::string algorithmTypeToString(mps::planner::sorting::PlanningProblem::AlgorithmType algo_type);

                mps::planner::sorting::PlanningProblem::AlgorithmType stringToSortingAlgorithmType(const std::string& str);
                mps::planner::sorting::PlanningProblem::ValueFunctionType stringToValueFnType(const std::string& str);

                void configurePlanningProblem(mps::planner::sorting::PlanningProblem& problem,
                                              const SortingPlanningProblemDesc& problem_desc);
            }
        }
    }
}

namespace YAML {
    template<>
    struct convert<mps::planner::util::yaml::SortingPlanningProblemDesc> {
        static Node encode(const mps::planner::util::yaml::SortingPlanningProblemDesc &problem_desc) {
            Node node;
            node["world_file"] = problem_desc.world_file;
            node["x_limits"] = problem_desc.x_limits;
            node["y_limits"] = problem_desc.y_limits;
            node["z_limits"] = problem_desc.z_limits;
            node["max_velocity"] = problem_desc.max_velocity;
            node["max_rotation_vel"] = problem_desc.max_rotation_vel;
            node["planning_timeout"] = problem_desc.planning_timeout;
            node["control_limits"] = problem_desc.control_limits;
            node["collision_policy"] = problem_desc.collision_policy;
            node["t_max"] = problem_desc.t_max;
            node["algorithm_type"] = mps::planner::util::yaml::algorithmTypeToString(problem_desc.algorithm_type);
            node["value_fn_type"] = mps::planner::util::yaml::valueFnTypeToString(problem_desc.value_fn_type);
            node["sorting_groups"] = problem_desc.sorting_groups;
            node["num_control_samples"] = problem_desc.num_control_samples;
            return node;
        }

        static bool decode(const Node &node, mps::planner::util::yaml::SortingPlanningProblemDesc &problem_desc) {
            problem_desc.world_file = node["world_file"].as<std::string>();
            problem_desc.x_limits = node["x_limits"].as<Eigen::Array2f>();
            problem_desc.y_limits = node["y_limits"].as<Eigen::Array2f>();
            problem_desc.z_limits = node["z_limits"].as<Eigen::Array2f>();
            problem_desc.max_velocity = node["max_velocity"].as<float>();
            problem_desc.max_rotation_vel = node["max_rotation_vel"].as<float>();
            problem_desc.planning_timeout = node["planning_timeout"].as<float>();
            problem_desc.control_limits = node["control_limits"].as<mps::planner::util::yaml::ControlLimitsDesc>();
            problem_desc.collision_policy = node["collision_policy"].as<mps::planner::util::yaml::CollisionPolicyDesc>();
            problem_desc.t_max = node["t_max"].as<float>();
            problem_desc.algorithm_type = mps::planner::util::yaml::stringToSortingAlgorithmType(node["algorithm_type"].as<std::string>());
            problem_desc.value_fn_type = mps::planner::util::yaml::stringToValueFnType(node["value_fn_type"].as<std::string>());
            problem_desc.sorting_groups = node["sorting_groups"].as<std::map<std::string, unsigned int> >();
            if (node["num_control_samples"]) {
                problem_desc.num_control_samples = node["num_control_samples"].as<unsigned int>();
            } else {
                problem_desc.num_control_samples = 1;
            }
            return true;
        }
    };
}

#endif //MANIPULATION_PLANNING_SUITE_ORACLEPARSING_H
