//
// Created by joshua on 9/11/17.
//

#ifndef MANIPULATION_PLANNING_SUITE_ORACLEPARSING_H
#define MANIPULATION_PLANNING_SUITE_ORACLEPARSING_H

#include <mps/planner/pushing/OraclePushPlanner.h>
#include <sim_env/utils/YamlUtils.h>
#include <yaml-cpp/yaml.h>
#include <Eigen/Core>

namespace mps {
    namespace planner {
        namespace util {
            namespace yaml {

                struct ControlLimitsDesc {
                    Eigen::VectorXf velocity_limits;
                    Eigen::VectorXf acceleration_limits;
                    Eigen::Array2f duration_limits;
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
                    float t_max;
                    Eigen::Vector3f goal_position;
                    float goal_region_radius;
                    mps::planner::pushing::PlanningProblem::OracleType oracle_type;
                    unsigned int num_control_samples;
                };

                std::string oracleTypeToString(mps::planner::pushing::PlanningProblem::OracleType oracle_type);

                mps::planner::pushing::PlanningProblem::OracleType stringToOracleType(const std::string& str);
            }
        }
    }
}

namespace YAML {
    template<>
    struct convert<mps::planner::util::yaml::ControlLimitsDesc> {
        static Node encode(const mps::planner::util::yaml::ControlLimitsDesc &control_desc) {
            Node node;
            node["velocity_limits"] = control_desc.velocity_limits;
            node["acceleration_limits"] = control_desc.acceleration_limits;
            node["duration_limits"] = control_desc.duration_limits;
            return node;
        }

        static bool decode(const Node &node, mps::planner::util::yaml::ControlLimitsDesc &control_desc) {
            control_desc.velocity_limits = node["velocity_limits"].as<Eigen::VectorXf>();
            control_desc.acceleration_limits = node["acceleration_limits"].as<Eigen::VectorXf>();
            control_desc.duration_limits = node["duration_limits"].as<Eigen::VectorXf>();
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
            node["goal_position"] = problem_desc.goal_position;
            node["goal_region_radius"] = problem_desc.goal_region_radius;
            node["oracle_type"] = mps::planner::util::yaml::oracleTypeToString(problem_desc.oracle_type);
            node["num_control_samples"] = problem_desc.num_control_samples;
            return node;
        }

        static bool decode(const Node &node, mps::planner::util::yaml::OraclePlanningProblemDesc &problem_desc) {
            problem_desc.world_file = node["world_file"].as<std::string>();
            problem_desc.robot_name = node["robot_name"].as<std::string>();
            problem_desc.target_name = node["target_name"].as<std::string>();
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
            problem_desc.goal_position = node["goal_position"].as<Eigen::VectorXf>();
            problem_desc.goal_region_radius = node["goal_region_radius"].as<float>();
            problem_desc.oracle_type = mps::planner::util::yaml::stringToOracleType(node["oracle_type"].as<std::string>());
            problem_desc.num_control_samples = node["num_control_samples"].as<unsigned int>();
            return true;
        }
    };
}

#endif //MANIPULATION_PLANNING_SUITE_ORACLEPARSING_H
