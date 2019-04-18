#include <boost/math/constants/constants.hpp>
#include <cmath>
#include <mps/planner/pushing/oracle/HumanOracle.h>
#include <mps/planner/util/Logging.h>
#include <mps/planner/util/Math.h>

using namespace mps::planner::pushing::oracle;
namespace b_math = boost::math;
namespace mps_logging = mps::planner::util::logging;

HumanOracle::Parameters::Parameters()
{
    robot_pose[0] = -0.08;
    robot_pose[1] = 0.0;
    robot_pose[2] = -1.57;
    alpha = 1.97;
    sig_trans = 0.02;
    sig_rot = 0.2;
    // pushability_covariance.setIdentity();
    // pushability_covariance(0, 0) = 0.1;
    // pushability_covariance(1, 1) = 0.1;
    // pushability_covariance(2, 2) = 0.78;
    // optimal_push_distance = 0.2;
    // push_distance_tolerance = 0.04;
    // push_angle_tolerance = 0.03;
}

// void HumanOracle::Parameters::computeInverses()
// {
//     _inv_pushability_covariance = pushability_covariance.inverse();
// }

HumanOracle::HumanOracle(RobotOraclePtr robot_oracle,
    unsigned int robot_id,
    const std::vector<sim_env::ObjectPtr>& objects,
    const Parameters& params)
    : _params(params)
    , _robot_steerer(robot_oracle)
    , _robot_id(robot_id)
{
    // _params.computeInverses();
    _rng = mps::planner::util::random::getDefaultRandomGenerator();
}

HumanOracle::~HumanOracle() = default;

void HumanOracle::predictAction(
    const mps::planner::ompl::state::SimEnvWorldState* current_state,
    const mps::planner::ompl::state::SimEnvWorldState* target_state,
    const unsigned int& obj_id, ::ompl::control::Control* control)
{
    static const std::string log_prefix("[HumanOracle::predictAction]");
    current_state->getObjectState(obj_id)->getConfiguration(_current_obj_state);
    current_state->getObjectState(_robot_id)->getConfiguration(_robot_state);
    target_state->getObjectState(obj_id)->getConfiguration(_next_obj_state);
    Eigen::Vector3f rel_obj(relativeSE2(_next_obj_state, _current_obj_state));
    float dtheta_r = _rng->gaussian(rel_obj[2], _params.sig_rot);
    _next_robot_state.resize(3);
    _next_robot_state.head(2) = _robot_state.head(2);
    _next_robot_state[2] = _robot_state[2] + dtheta_r;
    float cart_dist = rel_obj.head(2).norm();
    if (cart_dist != 0.0) {
        _next_robot_state.head(2) = _next_obj_state.head(2);
        _next_robot_state.head(2) += Eigen::Rotation2D(dtheta_r) * (_robot_state.head(2) - _current_obj_state.head(2));
    }
    _robot_steerer->steer(_robot_state, _next_robot_state, control);

    // OLD HUMAN ORACLE
    // Eigen::Vector3f next_robot_state(next_obj_state[0], next_obj_state[1], current_robot_state[2]);
    // if (rel_obj.head(2).norm() != 0.0) {
    //     Eigen::Vector2f heading = rel_obj.head(2) / rel_obj.head(2).norm();
    //     // we essentially move the robot as far as the next object state is away from the current object state.
    //     // since the current robot state must be with some offset to the object, we will not overshoot
    //     // Move robot to object goal state, but subtract half of object size
    //     float obj_size = fmax(_object_data[obj_id].width, _object_data[obj_id].height);
    //     next_robot_state.head(2) -= heading * obj_size / 2.0;
    //     std::vector<Eigen::VectorXf> controls;
    //     _robot_steerer->steer(current_robot_state, next_robot_state, controls);
    //     assert(not controls.empty());
    //     control = controls.at(0);
    // } else {
    //     mps_logging::logWarn("HumanOracle was asked to provide an action for non-translational push. "
    //                          "This oracle is not capable of dealing with this, returning null action",
    //         log_prefix);
    //     control.setZero(5);
    // }
}

void HumanOracle::samplePushingState(const mps::planner::ompl::state::SimEnvWorldState* current_state,
    const mps::planner::ompl::state::SimEnvWorldState* next_state,
    const unsigned int& obj_id,
    mps::planner::ompl::state::SimEnvObjectState* new_robot_state)
{
    current_state->getObjectState(obj_id)->getConfiguration(_current_obj_state);
    current_state->getObjectState(_robot_id)->getConfiguration(_robot_state);
    next_state->getObjectState(obj_id)->getConfiguration(_next_obj_state);
    Eigen::Vector3f se2_push_dir = relativeSE2(_next_obj_state, _current_obj_state);
    Eigen::Vector2f cart_dir(se2_push_dir[0], se2_push_dir[1]);
    cart_dir.normalize();
    // compute where on the partial circle around the object to place the robot
    float beta = 2.0f * _rng->uniform01() * _params.alpha - _params.alpha;
    // compute position of robot relative to object
    Eigen::Vector2f robot_pos = Eigen::Rotation2D(beta) * _params.robot_pose.head(2);
    // transform robot pose to global frame
    float gamma = std::atan2(cart_dir[1], cart_dir[0]);
    robot_pos = Eigen::Rotation2D(gamma) * robot_pos + _current_obj_state.head(2);
    float robot_orientation = beta + gamma + _params.robot_pose[2];
    // save state
    _next_robot_state.resize(3);
    _next_robot_state.head(2) = robot_pos;
    _next_robot_state[2] = robot_orientation;
    new_robot_state->setConfiguration(_next_robot_state);
    _next_robot_state.setZero();
    if (new_robot_state->hasVelocity()) {
        new_robot_state->setVelocity(_next_robot_state);
    }

    // OLD HUMAN ORACLE
    // double pushing_dist = _rng->gaussian(_params.optimal_push_distance, _params.push_distance_tolerance);
    // double angle = _rng->gaussian(0.0, _params.push_angle_tolerance);
    // Eigen::Vector3f rel_obj(relativeSE2(next_obj_state, current_obj_state));
    // if (rel_obj.norm() == 0.0) {
    //     rel_obj[0] = 0.0001 * (0.5f - _rng->uniform01());
    //     rel_obj[1] = 0.0001 * (0.5f - _rng->uniform01());
    // }
    // Eigen::Vector2f pushing_dir = rel_obj.head(2).normalized();
    // Eigen::Rotation2Df rotation((float)angle);
    // if (new_robot_state.size() < 3)
    //     new_robot_state.resize(3);
    // // the idea is to be offset from the current_obj config by pushing_dist and angle
    // new_robot_state.head(2) = current_obj_state.head(2);
    // Eigen::Vector2f pushing_offset = pushing_dist * rotation.toRotationMatrix() * pushing_dir;
    // new_robot_state.head(2) -= pushing_offset.head(2);
    // // orient the robot so that it faces into the desired pushing direction
    // new_robot_state[2] = std::atan2(pushing_dir[1], pushing_dir[0]) - 1.57f; // TODO the -1.57 is specific to the floating end-effector we use
}

Eigen::Vector3f HumanOracle::relativeSE2(const Eigen::VectorXf& state, const Eigen::VectorXf& ref)
{
    assert(state.size() >= 3);
    assert(ref.size() >= 3);
    Eigen::Vector3f dir(state[0] - ref[0], state[1] - ref[1], state[2] - ref[2]);
    dir[2] = mps::planner::util::math::shortest_direction_so2(ref[2], state[2]);
    return dir;
}
