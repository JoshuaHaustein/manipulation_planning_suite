#include <mps/planner/pushing/oracle/HumanOracle.h>
#include <boost/math/constants/constants.hpp>

using namespace mps::planner::pushing::oracle;
namespace b_math = boost::math;

HumanOracle::Parameters::Parameters() {
    pushability_covariance.setIdentity();
    pushability_covariance(0, 0) = 0.1;
    pushability_covariance(1, 1) = 0.1;
    pushability_covariance(2, 2) = 0.78;
    optimal_push_distance = 0.1;
    push_distance_tolerance = 0.03;
    push_angle_tolerance = 0.3;
}

void HumanOracle::Parameters::computeInverses() {
    _inv_pushability_covariance = pushability_covariance.inverse();
}

HumanOracle::HumanOracle(RampComputerPtr ramp_computer,
                         const Parameters& params) :
        _params(params),
        _ramp_computer(ramp_computer)
{
    _params.computeInverses();
    _rng = mps::planner::util::random::getDefaultRandomGenerator();
}

HumanOracle::~HumanOracle() = default;

void HumanOracle::prepareOracle(const Eigen::VectorXf &current_robot_state,
                                const Eigen::VectorXf &current_obj_state,
                                const Eigen::VectorXf &next_obj_state)
{
    // nothing to do
}

float HumanOracle::predictPushability(const Eigen::VectorXf &current_obj_state,
                                      const Eigen::VectorXf &next_obj_state) {
    assert(current_obj_state.size() == 3);
    assert(current_obj_state.size() == next_obj_state.size());
    // todo we assume object state is in SE(2)
    Eigen::Vector3f rel_se2(relativeSE2(next_obj_state, current_obj_state));
    // TODO maybe it's better to make this a pure function of the distance
    float maha_dist = std::sqrt(rel_se2.transpose().dot(_params._inv_pushability_covariance * rel_se2));
    if (maha_dist == 0.0) return std::numeric_limits<float>::max();
    return 1.0f / maha_dist;
}

float HumanOracle::predictFeasibility(const Eigen::VectorXf &current_robot_state,
                                      const Eigen::VectorXf &current_obj_state,
                                      const Eigen::VectorXf &next_obj_state) {
    assert(current_obj_state.size() == 3 && next_obj_state.size() == 3);
    Eigen::Vector3f obj_diff(relativeSE2(next_obj_state, current_obj_state));
    Eigen::Vector3f rel_robot(relativeSE2(current_robot_state, current_obj_state));
    Eigen::Vector2f push_dir(obj_diff.head(2));
    Eigen::Vector2f approach_dir(rel_robot.head(2));
    float robot_dist = approach_dir.norm();
    float object_dist_change = push_dir.norm();
    if (robot_dist == 0.0f)
    {
        // the robot is already colliding with the object, this is an infeasible state anyways
        return 0.0f;
    }
    float approach_angle;
    if (object_dist_change == 0.0f) {
        // we do not need to move the object, but maybe reorient it, as a simple heuristic set the approach angle to 0.0
        approach_angle = 0.0f;
    } else {
        approach_angle = std::acos(approach_dir.dot(push_dir) / (robot_dist * object_dist_change));
    }
    float args = std::pow(robot_dist - _params.optimal_push_distance, 2.0f) /
                 (2.0f * std::pow(_params.push_distance_tolerance, 2.0f));
    float distance_feasibility = std::exp(-args);
    args = std::pow(approach_angle, 2.0f) /
           (2.0f * std::pow(_params.push_angle_tolerance, 2.0f));
    float angle_feasibility = std::exp(-args);
    return distance_feasibility * angle_feasibility;
}

void HumanOracle::predictAction(const Eigen::VectorXf &current_robot_state,
                                const Eigen::VectorXf &current_obj_state,
                                const Eigen::VectorXf &next_obj_state,
                                Eigen::VectorXf &control)
{
    Eigen::Vector3f rel_obj(relativeSE2(next_obj_state, current_obj_state));
    Eigen::Vector3f next_robot_state(current_robot_state[0], current_robot_state[1], current_robot_state[2]);
    // we essentially move the robot as far as the next object state is away from the current object state.
    // since the current robot state must be with some offset to the object, we will not overshoot
    next_robot_state.head(2) += rel_obj.head(2);
    std::vector<Eigen::VectorXf> controls;
    _ramp_computer->steer(current_robot_state, next_robot_state, controls);
    assert(not controls.empty());
    control = controls.at(0);
}

void HumanOracle::sampleFeasibleState(Eigen::VectorXf &new_robot_state,
                                      const Eigen::VectorXf &current_obj_state,
                                      const Eigen::VectorXf &next_obj_state) {
    double pushing_dist = _rng->gaussian(_params.optimal_push_distance, _params.push_distance_tolerance);
    double angle = _rng->gaussian(0.0, _params.push_angle_tolerance);
    Eigen::Vector3f rel_obj(relativeSE2(next_obj_state, current_obj_state));
    Eigen::Vector2f pushing_dir = rel_obj.head(2).normalized();
    Eigen::Rotation2Df rotation((float)angle);
    if (new_robot_state.size() < 3) new_robot_state.resize(3);
    // the idea is to be offset from the current_obj config by pushing_dist and angle
    new_robot_state.head(2) = current_obj_state.head(2);
    Eigen::Vector2f pushing_offset = pushing_dist * rotation.toRotationMatrix() * pushing_dir;
    new_robot_state.head(2) -= pushing_offset.head(2);
    // orient the robot so that it faces into the desired pushing direction
    new_robot_state[2] = std::acos(rel_obj[0] / rel_obj.head(2).norm());
}

Eigen::Vector3f HumanOracle::relativeSE2(const Eigen::VectorXf& state, const Eigen::VectorXf& ref) {
    assert(state.size() >= 3);
    assert(ref.size() >= 3);
    Eigen::Vector3f dir(ref[0] - state[0], ref[1] - state[1], ref[2] - state[2]);
    if (dir[2] > b_math::constants::pi<float>()) {
        dir[2] -= 2.0f * b_math::constants::pi<float>();
    }
    return dir;
}
