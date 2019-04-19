//
// Created by joshua on 9/7/17.
//
// Modified by:
// - Isac 28/9/17
//
#include <csignal>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <mps/planner/ompl/control/RampVelocityControl.h>
#include <mps/planner/pushing/oracle/LearnedOracle.h>
#include <mps/planner/util/Logging.h>
#include <proto/oracle.pb.h>

namespace mps_logging = mps::planner::util::logging;
namespace mps_control = mps::planner::ompl::control;

mps::planner::pushing::oracle::LearnedPipeOracle::LearnedPipeOracle(const std::vector<sim_env::ObjectPtr>& objects, unsigned int robot_id)
    : _action_request_path(std::getenv("ORACLE_REQUEST_PIPE_PATH"))
    , _action_response_path(std::getenv("ORACLE_RESPONSE_PIPE_PATH"))
    , _state_sample_request_path(std::getenv("FEASIBILITY_SAMPLE_REQUEST_PIPE_PATH"))
    , _state_sample_response_path(std::getenv("FEASIBILITY_SAMPLE_RESPONSE_PIPE_PATH"))
    , _robot_id(robot_id)
{
    static const std::string log_prefix("[mps::planner::pushing::oracle::LearnedPipeOracle::LearnedPipeOracle]");
    GOOGLE_PROTOBUF_VERIFY_VERSION;
    if (!_action_request_path || !_action_response_path || !_state_sample_request_path || !_state_sample_response_path) {
        mps_logging::logErr("ERROR: Environment variables for oracle pipes not set", log_prefix);
        raise(SIGABRT);
    }
    // init object data
    for (unsigned int i = 0; i < objects.size(); ++i) {
        ObjectData data;
        auto object = objects.at(i);
        data.mass = object->getMass();
        data.inertia = object->getInertia();
        data.mu = object->getGroundFriction();
        auto aabb = object->getLocalAABB();
        data.width = aabb.getWidth();
        data.height = aabb.getHeight();
        _object_data.push_back(data);
    }
}

mps::planner::pushing::oracle::LearnedPipeOracle::~LearnedPipeOracle()
{
    /* No members allocated within the class at the moment */
}

void mps::planner::pushing::oracle::LearnedPipeOracle::samplePushingState(
    const mps::planner::ompl::state::SimEnvWorldState* current_state,
    const mps::planner::ompl::state::SimEnvWorldState* next_state,
    const unsigned int& obj_id,
    mps::planner::ompl::state::SimEnvObjectState* new_robot_state)
{
    // read configurations from state
    current_state->getObjectState(obj_id)->getConfiguration(_current_obj_state);
    next_state->getObjectState(obj_id)->getConfiguration(_target_obj_state);
    /* Create buffer */
    auto request = oracle_communication::FeasibilityRequest();

    // Initial state
    request.set_robot_x(0.0);
    request.set_robot_y(0.0);
    request.set_robot_radians(0.0);

    request.set_object_x(_current_obj_state[0]);
    request.set_object_y(_current_obj_state[1]);
    request.set_object_radians(_current_obj_state[2]);

    // Successor state
    request.set_object_x_prime(_target_obj_state[0]);
    request.set_object_y_prime(_target_obj_state[1]);
    request.set_object_radians_prime(_target_obj_state[2]);

    // Object parameters
    request.set_object_mass(_object_data[obj_id].mass);
    request.set_object_rotational_inertia(_object_data[obj_id].inertia);
    request.set_object_friction(_object_data[obj_id].mu);
    request.set_object_width(_object_data[obj_id].width);
    request.set_object_height(_object_data[obj_id].height);

    /* Send buffer over pipe */
    std::ofstream fd_out;
    fd_out.open(_state_sample_request_path, std::ios::out | std::ios::binary);
    request.SerializeToOstream(&fd_out);
    fd_out.close();

    /* Get response */
    auto response = oracle_communication::FeasibilityResponse();
    std::ifstream fd_in;
    fd_in.open(_state_sample_response_path, std::ios::in | std::ios::binary);
    response.ParseFromIstream(&fd_in);
    fd_in.close();

    _robot_state.resize(3);
    _robot_state[0] = response.robot_x();
    _robot_state[1] = response.robot_y();
    _robot_state[2] = response.robot_radians();
    new_robot_state->setConfiguration(_robot_state);
    if (new_robot_state->hasVelocity()) {
        _robot_state.setZero();
        new_robot_state->setVelocity(_robot_state);
    }

    if (timer) {
        timer->addExternalElapsedTime(response.cpu_time());
    }

    return;
}

void mps::planner::pushing::oracle::LearnedPipeOracle::predictAction(
    const mps::planner::ompl::state::SimEnvWorldState* current_state,
    const mps::planner::ompl::state::SimEnvWorldState* target_state,
    const unsigned int& obj_id, ::ompl::control::Control* control)
{
    /* Create buffer */
    auto request = oracle_communication::ActionRequest();

    // Initial state
    current_state->getObjectState(_robot_id)->getConfiguration(_robot_state);
    request.set_robot_x(_robot_state[0]);
    request.set_robot_y(_robot_state[1]);
    request.set_robot_radians(_robot_state[2]);

    current_state->getObjectState(obj_id)->getConfiguration(_current_obj_state);
    request.set_object_x(_current_obj_state[0]);
    request.set_object_y(_current_obj_state[1]);
    request.set_object_radians(_current_obj_state[2]);

    // Successor state
    target_state->getObjectState(obj_id)->getConfiguration(_target_obj_state);
    request.set_object_x_prime(_target_obj_state[0]);
    request.set_object_y_prime(_target_obj_state[1]);
    request.set_object_radians_prime(_target_obj_state[2]);

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

    _control.resize(5);
    _control[0] = response.dx();
    _control[1] = response.dy();
    _control[2] = response.dr();
    _control[3] = response.t();
    _control[4] = 0.0f;

    // TODO should also work for new control type
    // cast control to RampVelocityTwist
    auto ramp_twist = dynamic_cast<mps_control::RampVelocityControl*>(control);
    ramp_twist->setParameters(_control);

    if (timer) {
        timer->addExternalElapsedTime(response.cpu_time());
    }
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
