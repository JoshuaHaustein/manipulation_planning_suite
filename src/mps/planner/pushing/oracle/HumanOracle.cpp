#include <mps/planner/pushing/oracle/HumanOracle.h>
#include <boost/math/constants/constants.hpp>
#include <mps/planner/util/Math.h>
#include <mps/planner/util/Logging.h>

using namespace mps::planner::pushing::oracle;
namespace b_math = boost::math;
namespace mps_logging = mps::planner::util::logging;

HumanOracle::Parameters::Parameters() {
    pushability_covariance.setIdentity();
    pushability_covariance(0, 0) = 0.1;
    pushability_covariance(1, 1) = 0.1;
    pushability_covariance(2, 2) = 0.78;
    optimal_push_distance = 0.2;
    push_distance_tolerance = 0.04;
    push_angle_tolerance = 0.03;
}

void HumanOracle::Parameters::computeInverses() {
    _inv_pushability_covariance = pushability_covariance.inverse();
}

HumanOracle::HumanOracle(RobotOraclePtr robot_oracle,
                         const std::vector<ObjectData>& object_data,
                         const Parameters& params) :
        _params(params),
        _robot_steerer(robot_oracle)
{
    _params.computeInverses();
    _rng = mps::planner::util::random::getDefaultRandomGenerator();
    _object_data = object_data;
}

HumanOracle::~HumanOracle() = default;

float HumanOracle::predictPushability(const Eigen::VectorXf &current_obj_state,
                                      const Eigen::VectorXf &next_obj_state,
                                      const unsigned int& obj_id) {
    assert(current_obj_state.size() == 3);
    assert(current_obj_state.size() == next_obj_state.size());
    // todo we assume object state is in SE(2)
    Eigen::Vector3f rel_se2(relativeSE2(next_obj_state, current_obj_state));
    // TODO maybe it's better to make this a pure function of the distance
    float maha_dist = std::sqrt(rel_se2.transpose().dot(_params._inv_pushability_covariance * rel_se2));
    if (maha_dist == 0.0) return std::numeric_limits<float>::max();
    return 1.0f / maha_dist;
}

void HumanOracle::projectToPushability(const Eigen::VectorXf& current_obj_state,
                                       const Eigen::VectorXf& next_obj_state,
                                       const float& num_std,
                                       const unsigned int& obj_id,
                                       Eigen::VectorXf& output)
{
    static const std::string log_prefix("[mps::planner::oracle::HumanOracle::projectToPushability]");
    assert(current_obj_state.size() == 3);
    assert(next_obj_state.size() == 3);
    mps_logging::logDebug(boost::format("Projecting state %1% given current object state %2%") % next_obj_state.transpose() % current_obj_state.transpose(), log_prefix);
    output.conservativeResize(3);
    Eigen::Vector3f rel_se2(relativeSE2(next_obj_state, current_obj_state));
    // TODO Do we need to treat R^2 and SO(2) separately here?
    float maha_dist = std::sqrt(rel_se2.transpose().dot(_params._inv_pushability_covariance * rel_se2));
    if (maha_dist == 0.0f) {
        output = current_obj_state;
    }
    if (maha_dist > num_std) { // only do sth if the next object state is further away than num_std standard deviations
        for (unsigned int i = 0; i < 3; ++i) {
            output[i] = current_obj_state[i] + num_std * 1.0f / maha_dist * rel_se2[i];
        }
        mps::planner::util::math::normalize_orientation(output[2]);
    } else {
        output = next_obj_state;
    }
    mps_logging::logDebug(boost::format("Output state is %1%") % output.transpose(), log_prefix);
}

float HumanOracle::predictFeasibility(const Eigen::VectorXf &current_robot_state,
                                      const Eigen::VectorXf &current_obj_state,
                                      const Eigen::VectorXf &next_obj_state,
                                      const unsigned int& obj_id)
{
    assert(current_obj_state.size() == 3 && next_obj_state.size() == 3);
    Eigen::Vector3f obj_diff(relativeSE2(next_obj_state, current_obj_state));
    Eigen::Vector3f rel_robot(relativeSE2(current_robot_state, current_obj_state));
    Eigen::Vector2f push_dir(obj_diff.head(2));
    Eigen::Vector2f approach_dir(-rel_robot.head(2));
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
                                const unsigned int& obj_id,
                                Eigen::VectorXf &control)
{
    static const std::string log_prefix("[HumanOracle::predictAction]");
    Eigen::Vector3f rel_obj(relativeSE2(next_obj_state, current_obj_state));
    Eigen::Vector3f next_robot_state(next_obj_state[0], next_obj_state[1], current_robot_state[2]);
    if (rel_obj.head(2).norm() != 0.0) {
        Eigen::Vector2f heading = rel_obj.head(2) / rel_obj.head(2).norm();
        // we essentially move the robot as far as the next object state is away from the current object state.
        // since the current robot state must be with some offset to the object, we will not overshoot
        // Move robot to object goal state, but subtract half of object size
        float obj_size = fmax(_object_data[obj_id].width, _object_data[obj_id].height);
        next_robot_state.head(2) -= heading * obj_size / 2.0;
        std::vector<Eigen::VectorXf> controls;
        _robot_steerer->steer(current_robot_state, next_robot_state, controls);
        assert(not controls.empty());
        control = controls.at(0);
    } else {
        mps_logging::logWarn("HumanOracle was asked to provide an action for non-translational push. "
                             "This oracle is not capable of dealing with this, returning null action", log_prefix);
        control.setZero(4);
    }
}

void HumanOracle::sampleFeasibleState(const Eigen::VectorXf &current_obj_state,
                                      const Eigen::VectorXf &next_obj_state,
                                      const unsigned int& obj_id,
                                      Eigen::VectorXf &new_robot_state)
{
    double pushing_dist = _rng->gaussian(_params.optimal_push_distance, _params.push_distance_tolerance);
    double angle = _rng->gaussian(0.0, _params.push_angle_tolerance);
    Eigen::Vector3f rel_obj(relativeSE2(next_obj_state, current_obj_state));
    if (rel_obj.norm() == 0.0) {
        rel_obj[0] = 0.0001 * (0.5f - _rng->uniform01());
        rel_obj[1] = 0.0001 * (0.5f - _rng->uniform01());
    }
    Eigen::Vector2f pushing_dir = rel_obj.head(2).normalized();
    Eigen::Rotation2Df rotation((float)angle);
    if (new_robot_state.size() < 3) new_robot_state.resize(3);
    // the idea is to be offset from the current_obj config by pushing_dist and angle
    new_robot_state.head(2) = current_obj_state.head(2);
    Eigen::Vector2f pushing_offset = pushing_dist * rotation.toRotationMatrix() * pushing_dir;
    new_robot_state.head(2) -= pushing_offset.head(2);
    // orient the robot so that it faces into the desired pushing direction
    new_robot_state[2] = std::atan2(pushing_dir[1], pushing_dir[0]) - 1.57f; // TODO the -1.57 is specific to the floating end-effector we use
}

float HumanOracle::getMaximalPushingDistance() const {
    return std::sqrt(2.0f * _params.pushability_covariance(0,0) + 2.0f * _params.pushability_covariance(1,1))
            + 0.01f * std::sqrt(_params.pushability_covariance(2, 2));
}

Eigen::Vector3f HumanOracle::relativeSE2(const Eigen::VectorXf& state, const Eigen::VectorXf& ref) {
    assert(state.size() >= 3);
    assert(ref.size() >= 3);
    Eigen::Vector3f dir(state[0] - ref[0], state[1] - ref[1], state[2] - ref[2]);
    dir[2] = mps::planner::util::math::shortest_direction_so2(ref[2], state[2]);
    return dir;
}
