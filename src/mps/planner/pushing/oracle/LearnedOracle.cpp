//
// Created by joshua on 9/7/17.
//
// Modified by:
// - Isac 28/9/17
//
#include <mps/planner/pushing/oracle/LearnedOracle.h>
#include <mps/planner/pushing/oracle/oracle.pb.h>
#include <mps/planner/util/Logging.h>
#include <cstdlib>
#include <fstream>
#include <csignal>
#include <iostream>

namespace mps_logging = mps::planner::util::logging;

mps::planner::pushing::oracle::LearnedPipeOracle::LearnedPipeOracle(const std::vector<ObjectData>& object_data) :
    _action_request_path(std::getenv("ORACLE_REQUEST_PIPE_PATH")),
    _action_response_path(std::getenv("ORACLE_RESPONSE_PIPE_PATH")),
    _feasibility_request_path(std::getenv("FEASIBILITY_REQUEST_PIPE_PATH")),
    _feasibility_response_path(std::getenv("FEASIBILITY_RESPONSE_PIPE_PATH")),
    _feasibility_sample_request_path(std::getenv("FEASIBILITY_SAMPLE_REQUEST_PIPE_PATH")),
    _feasibility_sample_response_path(std::getenv("FEASIBILITY_SAMPLE_RESPONSE_PIPE_PATH")),
    _pushability_request_path(std::getenv("PUSHABILITY_REQUEST_PIPE_PATH")),
    _pushability_response_path(std::getenv("PUSHABILITY_RESPONSE_PIPE_PATH")),
    _pushability_projection_request_path(std::getenv("PUSHABILITY_PROJECTION_REQUEST_PIPE_PATH")),
    _pushability_projection_response_path(std::getenv("PUSHABILITY_PROJECTION_RESPONSE_PIPE_PATH"))
{
    static const std::string log_prefix("[mps::planner::pushing::oracle::LearnedPipeOracle::LearnedPipeOracle]");
    GOOGLE_PROTOBUF_VERIFY_VERSION;
    if (_feasibility_request_path == nullptr ||
        _feasibility_response_path == nullptr ||
        _action_request_path == nullptr ||
        _action_response_path == nullptr ||
        _feasibility_sample_request_path == nullptr ||
        _feasibility_sample_response_path == nullptr ||
        _pushability_projection_request_path == nullptr ||
        _pushability_projection_response_path == nullptr ||
        _pushability_request_path == nullptr ||
        _pushability_response_path == nullptr
        )
    {
        mps_logging::logDebug("ERROR: Environment variables for oracle pipes not set", log_prefix);
        raise(SIGABRT);
    }
    _object_data = object_data;
}

mps::planner::pushing::oracle::LearnedPipeOracle::~LearnedPipeOracle() {
    /* No members allocated within the class at the moment */
}

void mps::planner::pushing::oracle::LearnedPipeOracle::setObjectData(const std::vector<ObjectData>& object_data) {
    _object_data = object_data;
    return;
}

float mps::planner::pushing::oracle::LearnedPipeOracle::predictPushability(const Eigen::VectorXf &current_obj_state,
                         const Eigen::VectorXf &next_obj_state,
                         const unsigned int& obj_id) {
    /* Create buffer */
    auto request = oracle_communication::PushabilityRequest();

    // Not used for this request
    request.set_num_stds(-1.0);

    // Initial state
    request.set_object_x(current_obj_state[0]);
    request.set_object_y(current_obj_state[1]);
    request.set_object_radians(current_obj_state[2]);

    // Successor state
    request.set_object_x_prime(next_obj_state[0]);
    request.set_object_y_prime(next_obj_state[1]);
    request.set_object_radians_prime(next_obj_state[2]);

    // Object parameters
    request.set_object_mass(_object_data[obj_id].mass);
    request.set_object_rotational_inertia(_object_data[obj_id].inertia);
    request.set_object_friction(_object_data[obj_id].mu);
    request.set_object_width(_object_data[obj_id].width);
    request.set_object_height(_object_data[obj_id].height);

    /* Send buffer over pipe */
    std::ofstream fd_out;
    fd_out.open(_pushability_request_path, std::ios::out | std::ios::binary);
    request.SerializeToOstream(&fd_out);
    fd_out.close();

    /* Get response */
    auto response = oracle_communication::PushabilityResponse();
    std::ifstream fd_in;
    fd_in.open(_pushability_response_path, std::ios::in | std::ios::binary);
    response.ParseFromIstream(&fd_in);
    fd_in.close();
    return 1.0 / response.mahalanobis();
}

void mps::planner::pushing::oracle::LearnedPipeOracle::projectToPushability(const Eigen::VectorXf& current_obj_state,
                          const Eigen::VectorXf& next_obj_state,
                          const float& num_std,
                          const unsigned int& obj_id,
                          Eigen::VectorXf& output) {
    /* Create buffer */
    auto request = oracle_communication::PushabilityRequest();

    request.set_num_stds(num_std);

    // Initial state
    request.set_object_x(current_obj_state[0]);
    request.set_object_y(current_obj_state[1]);
    request.set_object_radians(current_obj_state[2]);

    // Successor state
    request.set_object_x_prime(next_obj_state[0]);
    request.set_object_y_prime(next_obj_state[1]);
    request.set_object_radians_prime(next_obj_state[2]);

    // Object parameters
    request.set_object_mass(_object_data[obj_id].mass);
    request.set_object_rotational_inertia(_object_data[obj_id].inertia);
    request.set_object_friction(_object_data[obj_id].mu);
    request.set_object_width(_object_data[obj_id].width);
    request.set_object_height(_object_data[obj_id].height);

    /* Send buffer over pipe */
    std::ofstream fd_out;
    fd_out.open(_pushability_projection_request_path, std::ios::out | std::ios::binary);
    request.SerializeToOstream(&fd_out);
    fd_out.close();

    /* Get response */
    auto response = oracle_communication::PushabilityResponse();
    std::ifstream fd_in;
    fd_in.open(_pushability_projection_response_path, std::ios::in | std::ios::binary);
    response.ParseFromIstream(&fd_in);
    fd_in.close();

    output[0] = response.projected_object_x();
    output[1] = response.projected_object_y();
    output[2] = response.projected_object_radians();

    return;
}

float mps::planner::pushing::oracle::LearnedPipeOracle::predictFeasibility(const Eigen::VectorXf &current_robot_state,
                         const Eigen::VectorXf &current_obj_state,
                         const Eigen::VectorXf &next_obj_state,
                         const unsigned int& obj_id) {
    /* Create buffer */
    auto request = oracle_communication::FeasibilityRequest();

    // Initial state
    request.set_robot_x(current_robot_state[0]);
    request.set_robot_y(current_robot_state[1]);
    request.set_robot_radians(current_robot_state[2]);

    request.set_object_x(current_obj_state[0]);
    request.set_object_y(current_obj_state[1]);
    request.set_object_radians(current_obj_state[2]);

    // Successor state
    request.set_object_x_prime(next_obj_state[0]);
    request.set_object_y_prime(next_obj_state[1]);
    request.set_object_radians_prime(next_obj_state[2]);

    // Object parameters
    request.set_object_mass(_object_data[obj_id].mass);
    request.set_object_rotational_inertia(_object_data[obj_id].inertia);
    request.set_object_friction(_object_data[obj_id].mu);
    request.set_object_width(_object_data[obj_id].width);
    request.set_object_height(_object_data[obj_id].height);

    /* Send buffer over pipe */
    std::ofstream fd_out;
    fd_out.open(_feasibility_request_path, std::ios::out | std::ios::binary);
    request.SerializeToOstream(&fd_out);
    fd_out.close();

    /* Get response */
    auto response = oracle_communication::FeasibilityResponse();
    std::ifstream fd_in;
    fd_in.open(_feasibility_response_path, std::ios::in | std::ios::binary);
    response.ParseFromIstream(&fd_in);
    fd_in.close();

    return 1.0 / response.mahalanobis();
}

void mps::planner::pushing::oracle::LearnedPipeOracle::sampleFeasibleState(const Eigen::VectorXf &current_obj_state,
                         const Eigen::VectorXf &next_obj_state,
                         const unsigned int& obj_id,
                         Eigen::VectorXf &new_robot_state) {
    /* Create buffer */
    auto request = oracle_communication::FeasibilityRequest();

    // Initial state
    request.set_robot_x(0.0);
    request.set_robot_y(0.0);
    request.set_robot_radians(0.0);

    request.set_object_x(current_obj_state[0]);
    request.set_object_y(current_obj_state[1]);
    request.set_object_radians(current_obj_state[2]);

    // Successor state
    request.set_object_x_prime(next_obj_state[0]);
    request.set_object_y_prime(next_obj_state[1]);
    request.set_object_radians_prime(next_obj_state[2]);

    // Object parameters
    request.set_object_mass(_object_data[obj_id].mass);
    request.set_object_rotational_inertia(_object_data[obj_id].inertia);
    request.set_object_friction(_object_data[obj_id].mu);
    request.set_object_width(_object_data[obj_id].width);
    request.set_object_height(_object_data[obj_id].height);

    /* Send buffer over pipe */
    std::ofstream fd_out;
    fd_out.open(_feasibility_sample_request_path, std::ios::out | std::ios::binary);
    request.SerializeToOstream(&fd_out);
    fd_out.close();

    /* Get response */
    auto response = oracle_communication::FeasibilityResponse();
    std::ifstream fd_in;
    fd_in.open(_feasibility_sample_response_path, std::ios::in | std::ios::binary);
    response.ParseFromIstream(&fd_in);
    fd_in.close();

    new_robot_state[0] = response.robot_x();
    new_robot_state[1] = response.robot_y();
    new_robot_state[2] = response.robot_radians();

    return;
}

void mps::planner::pushing::oracle::LearnedPipeOracle::predictAction(const Eigen::VectorXf &current_robot_state,
                   const Eigen::VectorXf &current_obj_state,
                   const Eigen::VectorXf &next_obj_state,
                   const unsigned int& obj_id,
                   Eigen::VectorXf &control) {
    /* Create buffer */
    auto request = oracle_communication::ActionRequest();

    // Initial state
    request.set_robot_x(current_robot_state[0]);
    request.set_robot_y(current_robot_state[1]);
    request.set_robot_radians(current_robot_state[2]);

    request.set_object_x(current_obj_state[0]);
    request.set_object_y(current_obj_state[1]);
    request.set_object_radians(current_obj_state[2]);

    // Successor state
    request.set_object_x_prime(next_obj_state[0]);
    request.set_object_y_prime(next_obj_state[1]);
    request.set_object_radians_prime(next_obj_state[2]);

    // Object parameters
    request.set_object_mass(_object_data[obj_id].mass);
    request.set_object_rotational_inertia(_object_data[obj_id].inertia);
    request.set_object_friction(_object_data[obj_id].mu);
    request.set_object_width(_object_data[obj_id].width);
    request.set_object_height(_object_data[obj_id].height);

    /* Send buffer over pipe */
    std::ofstream fd_out;
    fd_out.open(_action_request_path, std::ios::out | std::ios::binary);
    request.SerializeToOstream(&fd_out);
    fd_out.close();

    /* Get response */
    auto response = oracle_communication::ActionResponse();
    std::ifstream fd_in;
    fd_in.open(_action_response_path, std::ios::in | std::ios::binary);
    response.ParseFromIstream(&fd_in);
    fd_in.close();

    control[0] = response.dx();
    control[1] = response.dy();
    control[2] = response.dr();
    control[3] = response.t();

    return;
}

float mps::planner::pushing::oracle::LearnedPipeOracle::getMaximalPushingDistance() const {
    // TODO
    return 0.1f;
}

/* Keep as future playground? */
// int main() {
//     using namespace mps::planner::pushing::oracle;
//     auto object_data = std::vector<mps::planner::pushing::oracle::LearnedPipeOracle::ObjectData>();
//     mps::planner::pushing::oracle::LearnedPipeOracle::ObjectData obj1 = {
//         0.350, /* mass    */
//         0.002, /* inertia */
//         0.120, /* width   */
//         0.120, /* height  */
//         0.190  /* mu      */
//     };
// 
//     Eigen::VectorXf result (4);
// 
//     object_data.push_back(obj1);
//     float res_f;
// 
//     auto l  = mps::planner::pushing::oracle::LearnedPipeOracle(object_data);
//     for (int i = 0; i < 16; i++) {
//         Eigen::Vector3f current_robot_state (10.5, 10.5, 0.0);
//         Eigen::Vector3f current_obj_state (10.0, 10.0, 0.0);
//         Eigen::Vector3f next_obj_state (9.5, 9.5, 0.0);
//         l.predictAction(current_robot_state, current_obj_state, next_obj_state, 0, result);
//         printf("Action sample dx: %f dy: %f dθ: %f t: %f\n", result[0], result[1], result[2], result[3]);
//     }
//     return 0;
// }
