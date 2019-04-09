#include <boost/math/constants/constants.hpp>
#include <mps/planner/pushing/oracle/HumanOracle.h>
#include <mps/planner/util/Logging.h>
#include <mps/planner/util/Math.h>

using namespace mps::planner::pushing::oracle;
namespace b_math = boost::math;
namespace mps_logging = mps::planner::util::logging;

HumanOracle::Parameters::Parameters()
{
    // TODO
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
    const std::vector<ObjectData>& object_data,
    const Parameters& params)
    : _params(params)
    , _robot_steerer(robot_oracle)
{
    // _params.computeInverses();
    _rng = mps::planner::util::random::getDefaultRandomGenerator();
    _object_data = object_data;
}

HumanOracle::~HumanOracle() = default;

void HumanOracle::predictAction(const Eigen::VectorXf& current_robot_state,
    const Eigen::VectorXf& current_obj_state,
    const Eigen::VectorXf& next_obj_state,
    const unsigned int& obj_id,
    Eigen::VectorXf& control)
{
    static const std::string log_prefix("[HumanOracle::predictAction]");
    Eigen::Vector3f rel_obj(relativeSE2(next_obj_state, current_obj_state));
    Eigen::Vector3f next_robot_state(next_obj_state[0], next_obj_state[1], current_robot_state[2]);
    // OLD HUMAN ORACLE
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

void HumanOracle::samplePushingState(const Eigen::VectorXf& current_obj_state,
    const Eigen::VectorXf& next_obj_state,
    const unsigned int& obj_id,
    Eigen::VectorXf& new_robot_state)
{

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
