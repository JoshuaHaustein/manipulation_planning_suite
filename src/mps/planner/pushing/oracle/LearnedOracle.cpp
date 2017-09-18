//
// Created by joshua on 9/7/17.
//
#include <mps/planner/pushing/oracle/LearnedOracle.h>
#include <mps/planner/pushing/oracle/oracle.pb.h>
#include <mps/planner/util/Logging.h>
#include <cstdlib>
#include <fstream>
#include <csignal>
#include <iostream>

namespace mps_logging = mps::planner::util::logging;

mps::planner::pushing::oracle::LearnedPipeOracle::LearnedPipeOracle() :
    _action_request_path(std::getenv("ORACLE_REQUEST_PIPE_PATH")),
    _action_response_path(std::getenv("ORACLE_RESPONSE_PIPE_PATH")),
    _feasibility_request_path(std::getenv("FEASIBILITY_REQUEST_PIPE_PATH")),
    _feasibility_response_path(std::getenv("FEASIBILITY_RESPONSE_PIPE_PATH")),
    _feasibility_sample_request_path(std::getenv("FEASIBILITY_SAMPLE_REQUEST_PIPE_PATH")),
    _feasibility_sample_response_path(std::getenv("FEASIBILITY_SAMPLE_RESPONSE_PIPE_PATH")),
    _pushability_request_path(std::getenv("PUSHABILITY_REQUEST_PIPE_PATH")),
    _pushability_response_path(std::getenv("PUSHABILITY_RESPONSE_PIPE_PATH"))
{
    static const std::string log_prefix("[mps::planner::pushing::oracle::LearnedPipeOracle::LearnedPipeOracle]");
    GOOGLE_PROTOBUF_VERIFY_VERSION;
    if (_feasibility_request_path == nullptr ||
        _feasibility_response_path == nullptr ||
        _action_request_path == nullptr ||
        _action_response_path == nullptr ||
        _feasibility_sample_request_path == nullptr ||
        _feasibility_sample_response_path == nullptr ||
        _pushability_request_path == nullptr ||
        _pushability_response_path == nullptr
        )
    {
        mps_logging::logDebug("ERROR: Environment variables for oracle pipes not set", log_prefix);
        raise(SIGABRT);
    }
}

mps::planner::pushing::oracle::LearnedPipeOracle::~LearnedPipeOracle() = default;

void mps::planner::pushing::oracle::LearnedPipeOracle::prepareOracle(const Eigen::VectorXf &current_robot_state,
                                                                     const Eigen::VectorXf &current_obj_state,
                                                                     const Eigen::VectorXf &next_obj_state) {
}

float mps::planner::pushing::oracle::LearnedPipeOracle::predictPushability(const Eigen::VectorXf &current_obj_state,
                                                                           const Eigen::VectorXf &next_obj_state) {
    /* Create buffer */
    auto request = oracle_communication::PushabilityRequest();
    /* Initial state */
    request.set_object_radians(current_obj_state[2]);

    /* Successor state */
    request.set_object_relative_x_prime(next_obj_state[0] - current_obj_state[0]);
    request.set_object_relative_y_prime(next_obj_state[1] - current_obj_state[1]);
    request.set_object_radians_prime(next_obj_state[2]);

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
    return (float)(1.0 / (response.mahalanobis() + 1e-9));
}

void mps::planner::pushing::oracle::LearnedPipeOracle::projectToPushability(const Eigen::VectorXf& current_obj_state,
                                                                            const Eigen::VectorXf& next_obj_state,
                                                                            Eigen::VectorXf& output,
                                                                            float num_std)
{
    // TODO
}

float mps::planner::pushing::oracle::LearnedPipeOracle::predictFeasibility(const Eigen::VectorXf &current_robot_state,
                                                                           const Eigen::VectorXf &current_obj_state,
                                                                           const Eigen::VectorXf &next_obj_state) {
    static const std::string log_prefix("[mps::planner::pushing::oracle::LearnedPipeOracle::predictFeasibility]");
    /* Create buffer */
    auto request = oracle_communication::FeasibilityRequest();
    /* Initial state */
    request.set_robot_relative_x(current_robot_state[0] - current_obj_state[0]);
    request.set_robot_relative_y(current_robot_state[1] - current_obj_state[1]);
    request.set_robot_radians(current_robot_state[2]);
    request.set_object_radians(current_obj_state[2]);

    /* Successor state */
    request.set_object_relative_x_prime(next_obj_state[0] - current_obj_state[0]);
    request.set_object_relative_y_prime(next_obj_state[1] - current_obj_state[1]);
    request.set_object_radians_prime(next_obj_state[2]);

    /* Send buffer over pipe */
    std::ofstream fd_out;
    fd_out.open(_feasibility_request_path, std::ios::out | std::ios::binary);
    request.SerializeToOstream(&fd_out);
    fd_out.close();

    mps_logging::logDebug(boost::format("Relative robot x: %f") % request.robot_relative_x(), log_prefix);
    mps_logging::logDebug(boost::format("Relative robot y: %f") % request.robot_relative_y(), log_prefix);
    mps_logging::logDebug(boost::format("Relative object x: %f") % request.object_relative_x_prime(), log_prefix);
    mps_logging::logDebug(boost::format("Relative object y: %f") % request.object_relative_y_prime(), log_prefix);

    /* Get response */
    auto response = oracle_communication::FeasibilityResponse();
    std::ifstream fd_in;
    fd_in.open(_feasibility_response_path, std::ios::in | std::ios::binary);
    response.ParseFromIstream(&fd_in);
    fd_in.close();
    return (float)(1.0 / (response.mahalanobis() + 1e-9));
}

void mps::planner::pushing::oracle::LearnedPipeOracle::predictAction(const Eigen::VectorXf &current_robot_state,
                                                                     const Eigen::VectorXf &current_obj_state,
                                                                     const Eigen::VectorXf &next_obj_state,
                                                                     Eigen::VectorXf& control) {
    /* Create buffer */
    auto request = oracle_communication::ActionRequest();
    /* Initial state */
    request.set_robot_relative_x(current_robot_state[0] - current_obj_state[0]);
    request.set_robot_relative_y(current_robot_state[1] - current_obj_state[1]);
    request.set_robot_radians(current_robot_state[2]);
    request.set_object_radians(current_obj_state[2]);

    /* Successor state */
    request.set_object_relative_x_prime(next_obj_state[0] - current_obj_state[0]);
    request.set_object_relative_y_prime(next_obj_state[1] - current_obj_state[1]);
    request.set_object_radians_prime(next_obj_state[2]);

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

    control.resize(5);
    control[0] = response.dx();
    control[1] = response.dy();
    control[2] = response.dr();
    control[3] = response.t();
    control[4] = 0.0f;
}

void mps::planner::pushing::oracle::LearnedPipeOracle::sampleFeasibleState(Eigen::VectorXf &new_robot_state,
                                                                           const Eigen::VectorXf &current_obj_state,
                                                                           const Eigen::VectorXf &next_obj_state) {
    /* Create buffer */
    auto request = oracle_communication::FeasibilitySampleRequest();
    /* Initial state */
    request.set_object_radians(current_obj_state[2]);

    /* Successor state */
    request.set_object_relative_x_prime(next_obj_state[0] - current_obj_state[0]);
    request.set_object_relative_y_prime(next_obj_state[1] - current_obj_state[1]);
    request.set_object_radians_prime(next_obj_state[2]);

    /* Send buffer over pipe */
    std::ofstream fd_out;
    fd_out.open(_feasibility_sample_request_path, std::ios::out | std::ios::binary);
    request.SerializeToOstream(&fd_out);
    fd_out.close();

    /* Get response */
    auto response = oracle_communication::FeasibilitySampleResponse();
    std::ifstream fd_in;
    fd_in.open(_feasibility_sample_response_path, std::ios::in | std::ios::binary);
    response.ParseFromIstream(&fd_in);
    fd_in.close();

    new_robot_state.resize(3);
    new_robot_state[0] = response.robot_relative_x() + current_obj_state[0];
    new_robot_state[1] = response.robot_relative_y() + current_obj_state[1];
    new_robot_state[2] = response.robot_radians();
}

//int main() {
//    auto l  = mps::planner::pushing::oracle::LearnedPipeOracle();
//    Eigen::Vector3f current_robot_state (0.0, 0.0, 0.0);
//    Eigen::Vector3f current_obj_state (4.0, 4.0, 0.0);
//    Eigen::Vector3f next_obj_state (2.0, 2.0, 0.0);
//    Eigen::VectorXf state (3);
//    for (int i = 0; i < 16; i++) {
//        l.sampleFeasibleState(state, current_obj_state, next_obj_state);
//        std::cout << "Feasible state:" << std::endl;
//        printf("x: %.2f y: %.2f θ: %.2f\n", state[0], state[1], state[2]);
//    }
//    return 0;
//}
