//
// Created by joshua on 9/11/17.
//

#ifndef MANIPULATION_PLANNING_SUITE_ORACLEPARSING_H
#define MANIPULATION_PLANNING_SUITE_ORACLEPARSING_H

#include <mps/planner/pushing/OraclePushPlanner.h>
#include <sim_env/utils/YamlUtils.h>
#include <yaml-cpp/yaml.h>
#include <Eigen/Core>
#include <vector>

namespace mps {
    namespace planner {
        namespace util {
            namespace yaml {

                struct CollisionPolicyDesc {
                    bool static_collisions_allowed;
                    std::vector<std::string> static_collisions_blacklist;
                    std::vector<std::pair<std::string, std::string> > collisions_blacklist;
                };

                struct ControlLimitsDesc {
                    Eigen::VectorXf velocity_limits;
                    Eigen::VectorXf acceleration_limits;
                    Eigen::Array2f duration_limits;
                    std::vector<Eigen::VectorXi> subspaces;
                };

                struct GoalDesc {
                    std::string obj_name;
                    Eigen::Vector3f goal_position;
                    float goal_region_radius;
                };

                struct OraclePlanningProblemDesc {
                    std::string world_file;
                    std::string robot_name;
                    std::string target_name;
                    Eigen::Array2f x_limits;
                    Eigen::Array2f y_limits;
                    Eigen::Array2f z_limits;
                    float max_velocity;
                    float max_rotation_vel;
                    float planning_timeout;
                    // todo object weights
                    // todo weight map
                    ControlLimitsDesc control_limits;
                    CollisionPolicyDesc collision_policy;
                    float t_max;
                    std::vector<GoalDesc> goals;
                    float goal_bias;
                    float target_bias;
                    float robot_bias;
                    mps::planner::pushing::PlanningProblem::OracleType oracle_type;
                    mps::planner::pushing::PlanningProblem::AlgorithmType algorithm_type;
                    mps::planner::pushing::PlanningProblem::LocalPlanner local_planner_type;
                    unsigned int num_control_samples;
                };

                std::string oracleTypeToString(mps::planner::pushing::PlanningProblem::OracleType oracle_type);
                std::string localPlannerTypeToString(mps::planner::pushing::PlanningProblem::LocalPlanner local_type);
                std::string algorithmTypeToString(mps::planner::pushing::PlanningProblem::AlgorithmType oracle_type);

                mps::planner::pushing::PlanningProblem::AlgorithmType stringToAlgorithmType(const std::string& str);
                mps::planner::pushing::PlanningProblem::OracleType stringToOracleType(const std::string& str);
                mps::planner::pushing::PlanningProblem::LocalPlanner stringToLocalPlannerType(const std::string& str);

                void configurePlanningProblem(mps::planner::pushing::PlanningProblem& problem,
                                              const OraclePlanningProblemDesc& problem_desc);
            }
        }
    }
}

namespace YAML {
    template<>
    struct convert<mps::planner::util::yaml::CollisionPolicyDesc> {
        static Node encode(const mps::planner::util::yaml::CollisionPolicyDesc& policy_desc) {
            Node node;
            node["static_collisions_allowed"] = policy_desc.static_collisions_allowed;
            node["static_collisions_blacklist"] = policy_desc.static_collisions_blacklist;
            node["collisions_blacklist"] = policy_desc.collisions_blacklist;
            return node;
        }

        static bool decode(const Node &node, mps::planner::util::yaml::CollisionPolicyDesc& policy_desc) {
            policy_desc.static_collisions_allowed = node["static_collisions_allowed"].as<bool>();
            policy_desc.static_collisions_blacklist = node["static_collisions_blacklist"].as<std::vector<std::string> >();
            policy_desc.collisions_blacklist = node["collisions_blacklist"].as<std::vector< std::pair<std::string, std::string> > >();
            return true;
        }
    };

    template<>
    struct convert<mps::planner::util::yaml::ControlLimitsDesc> {
        static Node encode(const mps::planner::util::yaml::ControlLimitsDesc &control_desc) {
            Node node;
            node["velocity_limits"] = control_desc.velocity_limits;
            node["acceleration_limits"] = control_desc.acceleration_limits;
            node["duration_limits"] = control_desc.duration_limits;
            node["subspaces"] = control_desc.subspaces;
            return node;
        }

        static bool decode(const Node &node, mps::planner::util::yaml::ControlLimitsDesc &control_desc) {
            control_desc.velocity_limits = node["velocity_limits"].as<Eigen::VectorXf>();
            control_desc.acceleration_limits = node["acceleration_limits"].as<Eigen::VectorXf>();
            control_desc.duration_limits = node["duration_limits"].as<Eigen::VectorXf>();
            if (node["subspaces"]) {
                control_desc.subspaces = node["subspaces"].as<std::vector<Eigen::VectorXi> >();
            }
            return true;
        }
    };

    template<>
    struct convert<mps::planner::util::yaml::GoalDesc> {
        static Node encode(const mps::planner::util::yaml::GoalDesc &goal_desc) {
            Node node;
            node["position"] = goal_desc.goal_position;
            node["radius"] = goal_desc.goal_region_radius;
            node["object_name"] = goal_desc.obj_name;
            return node;
        }

        static bool decode(const Node &node, mps::planner::util::yaml::GoalDesc &goal_desc) {
            goal_desc.goal_position = node["position"].as<Eigen::Vector3f>();
            goal_desc.goal_region_radius = node["radius"].as<float>();
            goal_desc.obj_name = node["object_name"].as<std::string>();
            return true;
        }
    };

    template<>
    struct convert<mps::planner::util::yaml::OraclePlanningProblemDesc> {
        static Node encode(const mps::planner::util::yaml::OraclePlanningProblemDesc &problem_desc) {
            Node node;
            node["world_file"] = problem_desc.world_file;
            node["robot_name"] = problem_desc.robot_name;
            node["target_name"] = problem_desc.target_name;
            node["collision_policy"] = problem_desc.collision_policy;
            node["x_limits"] = problem_desc.x_limits;
            node["y_limits"] = problem_desc.y_limits;
            node["z_limits"] = problem_desc.z_limits;
            node["max_velocity"] = problem_desc.max_velocity;
            node["max_rotation_vel"] = problem_desc.max_rotation_vel;
            node["planning_timeout"] = problem_desc.planning_timeout;
            // todo object weights
            // todo weight map
            node["control_limits"] = problem_desc.control_limits;
            node["t_max"] = problem_desc.t_max;
            node["robot_bias"] = problem_desc.robot_bias;
            node["target_bias"] = problem_desc.target_bias;
            node["goal_bias"] = problem_desc.goal_bias;
            node["goals"] = problem_desc.goals;
            node["oracle_type"] = mps::planner::util::yaml::oracleTypeToString(problem_desc.oracle_type);
            node["algorithm_type"] = mps::planner::util::yaml::algorithmTypeToString(problem_desc.algorithm_type);
            node["local_planner_type"] = mps::planner::util::yaml::localPlannerTypeToString(problem_desc.local_planner_type);
            node["num_control_samples"] = problem_desc.num_control_samples;
            return node;
        }

        static bool decode(const Node &node, mps::planner::util::yaml::OraclePlanningProblemDesc &problem_desc) {
            problem_desc.world_file = node["world_file"].as<std::string>();
            problem_desc.robot_name = node["robot_name"].as<std::string>();
            problem_desc.collision_policy = node["collision_policy"].as<mps::planner::util::yaml::CollisionPolicyDesc>();
            problem_desc.x_limits = node["x_limits"].as<Eigen::Array2f>();
            problem_desc.y_limits = node["y_limits"].as<Eigen::Array2f>();
            problem_desc.z_limits = node["z_limits"].as<Eigen::Array2f>();
            problem_desc.max_velocity = node["max_velocity"].as<float>();
            problem_desc.max_rotation_vel = node["max_rotation_vel"].as<float>();
            problem_desc.planning_timeout = node["planning_timeout"].as<float>();
            // todo object weights
            // todo weight map
            problem_desc.control_limits = node["control_limits"].as<mps::planner::util::yaml::ControlLimitsDesc>();
            problem_desc.t_max = node["t_max"].as<float>();
            for (auto yaml_goal : node["goals"]) {
                auto goal_desc = yaml_goal.as<mps::planner::util::yaml::GoalDesc>();
                problem_desc.goals.push_back(goal_desc);
            }
            problem_desc.oracle_type = mps::planner::util::yaml::stringToOracleType(node["oracle_type"].as<std::string>());
            problem_desc.algorithm_type = mps::planner::util::yaml::stringToAlgorithmType(node["algorithm_type"].as<std::string>());
            problem_desc.local_planner_type = mps::planner::util::yaml::stringToLocalPlannerType(node["local_planner_type"].as<std::string>());
            problem_desc.num_control_samples = node["num_control_samples"].as<unsigned int>();
            problem_desc.robot_bias = node["robot_bias"].as<float>();
            problem_desc.target_bias = node["target_bias"].as<float>();
            problem_desc.goal_bias = node["goal_bias"].as<float>();
            return true;
        }
    };
}

#endif //MANIPULATION_PLANNING_SUITE_ORACLEPARSING_H
