#include <boost/geometry.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <functional>
#include <mps/planner/ompl/control/TimedWaypoints.h>
#include <mps/planner/pushing/oracle/QuasiStaticSE2Oracle.h>
#include <mps/planner/util/Logging.h>
#include <mps/planner/util/Math.h>

namespace mps_state = mps::planner::ompl::state;
namespace mps_logging = mps::planner::util::logging;
namespace bg = boost::geometry;
namespace mps_control = mps::planner::ompl::control;
namespace mps_math = mps::planner::util::math;

using namespace mps::planner::pushing::oracle;

struct PushingVelocityConstraint {
    std::string obj_name;
    float max_vel;
    float heading_r; // angle of ur in object frame (in range [-pi, pi))
    float heading_l; // angle of ul (in range [-pi, pi))
    float r; // distance of contact point from robot origin
    float r_angle; // angle of r (r and r_angle are polar coordinates of the contact point in robot frame)

    void projectToCone(Eigen::VectorXf& robot_vel, sim_env::RobotConstPtr robot) const
    {
        // first get pose of the target object
        auto world = robot->getConstWorld();
        auto target_obj = world->getObjectConst(obj_name);
        assert(target_obj);
        auto obj_pose = target_obj->getDOFPositions();
        auto rob_pose = robot->getDOFPositions().head(3);
        // transform velocity constraint to world frame
        float heading_l_w = mps_math::normalize_orientation(heading_l + obj_pose[2]); // max angle
        float heading_r_w = mps_math::normalize_orientation(heading_r + obj_pose[2]); // min angle
        // compute velocity of pushing point (in world frame) given the commanded velocity
        Eigen::Vector2f tangential_vel = r * robot_vel[2] * Eigen::Vector2f(-std::sin(rob_pose[2] + r_angle), std::cos(rob_pose[2] + r_angle));
        Eigen::Vector2f vp(robot_vel[0], robot_vel[1]);
        vp += tangential_vel;
        // compute angle of vp
        // float vp_norm = vp.norm();
        float vp_angle = std::atan2(vp[1], vp[0]);
        // project angle into SO2 range [min_heading_w, max_heading_w)
        float vp_angle_prime = mps_math::projectToSO2Range(heading_r_w, heading_l_w, vp_angle);
        // mps_math::clampInplace(vp_angle, min_heading_w, max_heading_w);
        // proejct vp to direction of new vp
        Eigen::Vector2f new_vp(std::cos(vp_angle_prime), std::sin(vp_angle_prime));
        vp = vp.dot(new_vp) * new_vp;
        // clamp magnitude of velocity
        // mps_math::clampInplace(vp_norm, 0.0f, max_vel);
        // overwrite vp to what it is allowed to be
        // vp[0]
        //     = 1.5f * vp_norm * std::cos(vp_angle_prime);
        // vp[1] = 1.5f * vp_norm * std::sin(vp_angle_prime);
        // overwrite cartesian robot velocity
        // clamp magnitude of velocity
        Eigen::Vector2f updated_vel = vp - tangential_vel;
        float new_vel = updated_vel.norm();
        mps_math::clampInplace(new_vel, 0.0f, 1.5f * max_vel);
        updated_vel = new_vel * updated_vel.normalized();
        float delta_angle = 0.0f;
        float old_vel = robot_vel.head(2).norm();
        if (new_vel * old_vel > 0.0f) {
            delta_angle = updated_vel.dot(robot_vel.head(2)) / (new_vel * old_vel);
            delta_angle = std::acos(std::max(std::min(delta_angle, 1.0f), -1.0f));
        }
        float delta_norm = new_vel / old_vel;
        auto logger = world->getConstLogger();
        std::string log_prefix("[VelocityConstraint::projectToCone]");
        // logger->logDebug(boost::format("Change in velocity due to projection: Delta angle %1%, delta norm %2%") % delta_angle % delta_norm, "Projection");
        // logger->logDebug(boost::format("Tangential velocity: %1%, %2%") % tangential_vel[0] % tangential_vel[1], log_prefix);
        // logger->logDebug(boost::format("Heading bounds: %1%, %2%, Actual heading: %3%, Projected heading: %4%") % heading_l_w % heading_r_w % vp_angle % vp_angle_prime, log_prefix);
        if (vp_angle_prime != vp_angle) {
            logger->logWarn("Slippage predicted", log_prefix);
        }
        robot_vel.head(2) = updated_vel;
    }
};

QuasiStaticSE2Oracle::Parameters::Parameters()
    : eps_min(0.005f)
    , eps_dist(0.003f)
    , col_sample_step(0.0025f)
    , exp_weight(2.0f)
    , orientation_weight(0.1f)
    , path_step_size(0.01f)
    , push_vel(0.08f)
    , rot_push_vel(0.2f)
    , push_penetration(0.0f)
    , max_push_state_distance(1e-6)
    , max_obj_state_deviation(0.015f)
    , action_length(0.2f)
{
}

QuasiStaticSE2Oracle::PushingPathCache::PushingPathCache()
{
    seg_idx = 0;
    in_seg_idx = 0;
}

bool QuasiStaticSE2Oracle::PushingPathCache::nextSampleIdx()
{
    if (seg_idx == 3)
        return false;
    // forward to the next valid sample
    do {
        ++in_seg_idx;
        if (in_seg_idx >= num_samples[seg_idx]) {
            ++seg_idx;
            in_seg_idx = 0;
        }
    } while (in_seg_idx >= num_samples[seg_idx] and seg_idx < 3);
    return seg_idx < 3;
}

QuasiStaticSE2Oracle::PushingEdgeCache::PushingEdgeCache()
{
    edge_translation = 0.0;
    pair_id = 0;
}

QuasiStaticSE2Oracle::PushingCache::PushingCache()
{
    reset();
}

void QuasiStaticSE2Oracle::PushingCache::reset()
{
    path_computed = false;
    edge_pair_computed = false;
    obj_id = 0;
    ec = PushingEdgeCache();
    pc = PushingPathCache();
}

QuasiStaticSE2Oracle::QuasiStaticSE2Oracle(RobotOraclePtr robot_oracle,
    const std::vector<sim_env::ObjectPtr>& objects, unsigned int robot_id)
    : _robot_oracle(robot_oracle)
    , _robot_id(robot_id)
    , _objects(objects)
    , _random_gen(mps::planner::util::random::getDefaultRandomGenerator())
{
    _se2_state_a = dynamic_cast<ompl::state::QuasiStaticPushingStateSpace::StateType*>(_dubins_state_space.allocState());
    _se2_state_b = dynamic_cast<ompl::state::QuasiStaticPushingStateSpace::StateType*>(_dubins_state_space.allocState());
    _se2_state_c = dynamic_cast<ompl::state::QuasiStaticPushingStateSpace::StateType*>(_dubins_state_space.allocState());
    _eigen_config.resize(3);
    _eigen_config2.resize(3);
    computeRobotPushingEdges();
    computeObjectPushingEdges();
    // _tmp_edge_counter = 0;
    // _min_max_toggle = false;
}

QuasiStaticSE2Oracle::~QuasiStaticSE2Oracle()
{
    _dubins_state_space.freeState(_se2_state_a);
    _dubins_state_space.freeState(_se2_state_b);
    _dubins_state_space.freeState(_se2_state_c);
}

void QuasiStaticSE2Oracle::predictAction(const mps_state::SimEnvWorldState* current_state,
    const mps_state::SimEnvWorldState* target_state,
    const unsigned int& obj_id, ::ompl::control::Control* control)
{
    // TODO if the object is already at its target state, we don't need to do anything
    // initialize control
    auto* pos_control = dynamic_cast<mps_control::TimedWaypoints*>(control);
    assert(pos_control);
    pos_control->reset();
    // get object state
    current_state->getObjectState(_cache.obj_id)->getConfiguration(_eigen_config);
    Eigen::Vector3f obj_state(_eigen_config.head(3));
    // get target object state
    target_state->getObjectState(_cache.obj_id)->getConfiguration(_eigen_config);
    Eigen::Vector3f target_obj_state(_eigen_config.head(3));
    // get robot state
    current_state->getObjectState(_robot_id)->getConfiguration(_eigen_config);
    Eigen::Vector3f rob_state(_eigen_config.head(3));
    // check whether our cache has to be invalidated
    if (_cache.obj_id != obj_id or _cache.target_state != target_obj_state) {
        _cache.reset();
        _cache.target_state = target_obj_state;
    }
    // if we have a path, check whether we are on track and optionally prepend a re-approach action
    bool need_new_path = true;
    if (_cache.path_computed) {
        float obj_pos_error = (obj_state.head(2) - _cache.pc.obj_start_state.head(2)).norm();
        float obj_orientation_error = _params.orientation_weight * std::abs(mps_math::shortest_direction_so2(obj_state[2], _cache.pc.obj_start_state[2]));
        need_new_path = obj_pos_error + obj_orientation_error > _params.max_obj_state_deviation;
    }
    if (need_new_path) {
        // reset path cache
        _cache.path_computed = false;
        _cache.pc = PushingPathCache();
        mps_logging::logDebug("Computing new pushing path", "[QuasiStaticSE2Oracle::predictAction]");
        // we have no path yet, compute a new one
        // get pushing edge pair
        auto [closest_state, state_dist] = selectEdgePair(obj_id, current_state);
        // check if we need to approach the pushing state first
        if (state_dist > _params.max_push_state_distance) {
            mps_logging::logDebug("Too far away from pushing state, computing approach path", "[QuasiStaticSE2Oracle::predictAction]");
            // need to move to closest pushing state first
            current_state->getObjectState(_robot_id)->getConfiguration(_eigen_config);
            _eigen_config2.head(3) = closest_state;
            _robot_oracle->steer(_eigen_config, _eigen_config2, pos_control);
            if (state_dist > _params.action_length) {
                return;
            }
            rob_state = closest_state;
        }
        computePushingPath(obj_state, target_obj_state, rob_state);
    } else {
        // float robot_pose_error = (rob_state.head(2) - _cache.start_state.head(2)).norm() + _params.orientation_weight * std::abs(rob_state[2] - _cache.start_state[2]);
        // // if we the robot is too far off from its desired state, move it there first
        // if (robot_pose_error > _params.max_push_state_distance) {
        // }
        mps_logging::logDebug("Path cached. Sampling from existing path", "[QuasiStaticSE2Oracle::predictAction]");
    }
    samplePath(pos_control);
}

void QuasiStaticSE2Oracle::samplePushingState(const mps_state::SimEnvWorldState* current_state,
    const mps_state::SimEnvWorldState* next_state,
    const unsigned int& obj_id,
    mps_state::SimEnvObjectState* new_robot_state)
{
    static const std::string log_prefix("[QuasiStaticSE2Oracle::samplePushingState]");
    mps_logging::logDebug("samplePushingState", log_prefix);
    assert(obj_id < _contact_pairs.size());
    assert(obj_id != _robot_id);
    // compute sampling weights for each pair base on pushing direction
    std::vector<float> sampling_weights;
    float normalizer = computeSamplingWeights(current_state, next_state, obj_id, sampling_weights);
    auto& pushing_edge_pairs = _contact_pairs.at(obj_id);
    // sample pair
    unsigned int pair_id = pushing_edge_pairs.size() - 1;
    float die = _random_gen->uniform01();
    float acc = 0.0f;
    for (unsigned int id = 0; id < pushing_edge_pairs.size(); ++id) {
        acc += sampling_weights.at(id) / normalizer;
        if (die <= acc) {
            pair_id = id;
            break;
        }
    }
    // Helper variables for debugging
    // _tmp_edge_counter = _tmp_edge_counter % pushing_edge_pairs.size();
    // pair_id = _tmp_edge_counter;
    // _tmp_edge_counter += _min_max_toggle;
    // _min_max_toggle = !_min_max_toggle;
    // place the robot such that its pushing edge faces the object's pushing edge
    auto& edge_pair = pushing_edge_pairs.at(pair_id);
    float rel_trans = _random_gen->uniformReal(edge_pair.min_translation, edge_pair.max_translation);
    // float rel_trans = _min_max_toggle ? edge_pair.max_translation : edge_pair.min_translation;
    current_state->getObjectState(obj_id)->getConfiguration(_eigen_config);
    computeRobotState(_eigen_config2, edge_pair, _eigen_config, rel_trans, _params.eps_dist);
    new_robot_state->setConfiguration(_eigen_config2);
    auto logger = mps_logging::getLogger();
    // logger->logDebug(boost::format("Sampled pushing state %1%, %2%, %3%") % _eigen_config2[0] % _eigen_config2[1] % _eigen_config2[2], log_prefix);
    _eigen_config.setZero();
    new_robot_state->setVelocity(_eigen_config);
    // save contact pair that we used
    _cache.obj_id = obj_id;
    _cache.ec.pair_id = pair_id;
    _cache.ec.edge_translation = rel_trans;
    _cache.edge_pair_computed = true;
    next_state->getObjectState(obj_id)->getConfiguration(_eigen_config);
    _cache.target_state = _eigen_config.head(3);
}

/************************************** Policy Helper ************************************/
void QuasiStaticSE2Oracle::computePushingPath(const Eigen::Vector3f& obj_state, const Eigen::Vector3f& target_state, const Eigen::Vector3f& rob_state)
{
    // get edge pair
    PushingEdgePair& edge_pair = _contact_pairs.at(_cache.obj_id).at(_cache.ec.pair_id);
    // compute transformation matrices for current and target object state and current robot state
    Eigen::Affine2f wTo_c = Eigen::Translation2f(obj_state.head(2)) * Eigen::Rotation2Df(obj_state[2]);
    Eigen::Affine2f wTo_t = Eigen::Translation2f(target_state.head(2)) * Eigen::Rotation2Df(target_state[2]);
    // compute relative transform between robot and object
    Eigen::Affine2f wTr = Eigen::Translation2f(rob_state.head(2)) * Eigen::Rotation2Df(rob_state[2]);
    Eigen::Affine2f oTr = wTo_c.inverse() * wTr;
    // compute contact points
    // Eigen::Vector2f osr = oTr * edge_pair.robot_edge->from;
    // Eigen::Vector2f oer = oTr * edge_pair.robot_edge->to;
    // compute where the start and end point of the robot edge fall onto the object edge
    // float tsr = projectToEdge(osr, edge_pair.object_edge);
    // float ter = projectToEdge(oer, edge_pair.object_edge);
    // if (tsr > ter)
    //     std::swap(tsr, ter);
    // in case the robot edge is longer than the object edge, bound t to the object edge extensions
    // float tmin = std::max(0.0f, tsr);
    // float tmax = std::min(edge_pair.object_edge->edge_length, ter);
    // the reference contact point lies in the middle of the points object_edge.from + tmin * object_edge.dir
    // and object_edge.from + tmax * object_edge.dir
    // float t = (tmin + tmax) / 2.0f;
    // Eigen::Vector2f cp(edge_pair.object_edge->from + t * edge_pair.object_edge->dir);
    Eigen::Vector2f cp(edge_pair.object_edge->pcom);
    // define transform from pushing frame into object frame
    auto base_link = _objects.at(_cache.obj_id)->getBaseLink();
    Eigen::Vector3f com;
    base_link->getLocalCenterOfMass(com);
    Eigen::Vector2f pydir = com.head(2) - cp;
    float py = -pydir.norm();
    pydir.normalize();
    // float pushing_frame_angle = std::acos(edge_pair.object_edge->normal.dot(-pydir));
    float pushing_frame_angle = std::atan2(-pydir[0], pydir[1]); // angle between x axis of object and pushing frame
    Eigen::Affine2f oTp = Eigen::Translation2f(com.head(2)) * Eigen::Rotation2Df(pushing_frame_angle);
    // transform friction cone into pushing frame
    // TODO take friction from the link that is in contact (only needed for multi-link robots)
    float friction_coeff = std::sqrt(base_link->getContactFriction() * _objects.at(_robot_id)->getBaseLink()->getContactFriction());
    Eigen::Affine2f p_rot_e;
    p_rot_e = Eigen::Rotation2Df(-pushing_frame_angle + edge_pair.object_edge->edge_angle);
    Eigen::Vector2f fr = p_rot_e * Eigen::Vector2f(friction_coeff, 1.0f);
    Eigen::Vector2f fl = p_rot_e * Eigen::Vector2f(-friction_coeff, 1.0f);
    // get ground maximal friction wrench
    float max_force = base_link->getGroundFrictionLimitForce();
    float max_torque = base_link->getGroundFrictionLimitTorque();
    float a = 2.0f / (max_force * max_force);
    float b = 2.0f / (max_torque * max_torque);
    // compute z_l and z_r
    Eigen::Vector2f z_l(a * fl[1] / (b * py * fl[0]), a / (-b * py));
    Eigen::Vector2f z_r(a * fr[1] / (b * py * fr[0]), a / (-b * py));
    // compute z
    Eigen::Vector2f z((z_l[0] + z_r[0]) / 2.0f, a / (-b * py));
    float turning_radius = std::abs(z_l[0] - z_r[0]) / 2.0f;
    _dubins_state_space.setTurningRadius(turning_radius);
    // compute oTz and its inverse
    Eigen::Affine2f oTz = oTp * Eigen::Translation2f(z);
    // compute where z is in world frame given the current and target object pose
    Eigen::Affine2f wTz_c = wTo_c * oTz;
    Eigen::Affine2f wTz_t = wTo_t * oTz;
    computeAction(wTz_c, wTz_t, oTz.inverse() * oTr);
    // save start state
    _cache.pc.start_state = rob_state;
    _cache.pc.obj_start_state = obj_state;
    _cache.pc.rTo = oTr.inverse();
}

std::pair<Eigen::Vector3f, float> QuasiStaticSE2Oracle::selectEdgePair(unsigned int obj_id, const ompl::state::SimEnvWorldState* current_state)
{
    unsigned int pair_id = _contact_pairs.at(obj_id).size();
    float best_pair_distance = std::numeric_limits<float>::max();
    float translation = 0.0f;
    Eigen::Vector3f best_closest_state;
    // do we already have a contact edge?
    if (_cache.obj_id == obj_id and _cache.edge_pair_computed) {
        // check how close the two edges are in the current state
        auto [dist, ttrans] = computePushingStateDistance(obj_id, _cache.ec.pair_id, current_state, best_closest_state);
        // if this distance is already smaller than _params.eps_dist, just return this pair
        if (dist <= _params.eps_dist) {
            _cache.ec.edge_translation = ttrans;
            return { best_closest_state, dist };
        }
        // else we will search for the closest pair
        pair_id = _cache.ec.pair_id;
        best_pair_distance = dist;
        translation = ttrans;
    }
    // search for the closest pair and select that, TODO this could be done more efficiently with a GNAT tree
    for (unsigned int id = 0; id < _contact_pairs.at(obj_id).size(); ++id) {
        Eigen::Vector3f closest_state;
        auto [dist, ttrans] = computePushingStateDistance(obj_id, id, current_state, closest_state);
        if (dist < best_pair_distance) {
            best_pair_distance = dist;
            translation = ttrans;
            pair_id = id;
            best_closest_state = closest_state;
        }
    }
    _cache.edge_pair_computed = true;
    _cache.obj_id = obj_id;
    _cache.ec.pair_id = pair_id;
    _cache.ec.edge_translation = translation;
    return { best_closest_state, best_pair_distance };
}

std::pair<float, float> QuasiStaticSE2Oracle::computePushingStateDistance(unsigned int obj_id, unsigned int pair_id, const ompl::state::SimEnvWorldState* current_state,
    Eigen::Vector3f& closest_state) const
{
    auto& edge_pair = _contact_pairs.at(obj_id).at(pair_id);
    float translation_range = edge_pair.max_translation - edge_pair.min_translation;
    float translation = 0.0f;
    // get robot state
    current_state->getObjectState(_robot_id)->getConfiguration(_eigen_config);
    Eigen::Vector3f current_robot_state(_eigen_config.head(3));
    // get object state
    current_state->getObjectState(obj_id)->getConfiguration(_eigen_config);
    // compute distance to closest pushing state
    if (translation_range == 0.0f) { // there is just a single pushing state
        computeRobotState(_eigen_config, edge_pair, _eigen_config, edge_pair.max_translation, 0.0f);
        closest_state.head(2) = _eigen_config.head(2);
        closest_state[2] = _eigen_config[2];
    } else { // pushing states are forming a line
        // compute this line
        computeRobotState(_eigen_config2, edge_pair, _eigen_config, edge_pair.min_translation, 0.0f);
        Eigen::Vector2f start_pos(_eigen_config2[0], _eigen_config2[1]);
        computeRobotState(_eigen_config2, edge_pair, _eigen_config, edge_pair.max_translation, 0.0f);
        Eigen::Vector2f end_pos(_eigen_config2[0], _eigen_config2[1]);
        closest_state[2] = _eigen_config2[2];
        Eigen::Vector2f dir = (end_pos - start_pos).normalized();
        // compute normal
        Eigen::Vector2f normal(-dir[1], dir[0]);
        // project current position on line
        Eigen::Vector2f rel_pos(current_robot_state.head(2) - start_pos);
        Eigen::Vector2f on_line_pos = rel_pos - rel_pos.dot(normal) * normal;
        float sdist = on_line_pos.dot(dir);
        // check whether the projected point falls onto the line
        if (sdist < 0.0f) {
            closest_state.head(2) = start_pos.head(2);
            translation = edge_pair.min_translation;
        } else if (sdist > translation_range) {
            closest_state.head(2) = end_pos.head(2);
            translation = edge_pair.max_translation;
        } else {
            closest_state.head(2) = on_line_pos + start_pos;
            translation = edge_pair.min_translation + sdist;
        }
    }
    // compute distance to closest state
    // target state
    float angle_distance = std::abs(mps_math::shortest_direction_so2(closest_state[2], current_robot_state[2]));
    float cart_distance = (current_robot_state.head(2) - closest_state.head(2)).norm();
    return { cart_distance + _params.orientation_weight * angle_distance, translation };
}

// float QuasiStaticSE2Oracle::projectToEdge(const Eigen::Vector2f& point, const ObjectPushingEdgePtr ope) const
// {
//     Eigen::Vector2f rel_point = point - ope->from;
//     return (rel_point - rel_point.dot(ope->normal) * ope->normal).dot(ope->dir);
// }

void QuasiStaticSE2Oracle::computeAction(const Eigen::Affine2f& wTz_c, const Eigen::Affine2f& wTz_t,
    const Eigen::Affine2f& zTr) const
{
    const std::string log_prefix("[QuasiStaticSE2Oracle::computeAction]");
    // set start and target state for z
    _se2_state_a->setXY(wTz_c.translation().x(), wTz_c.translation().y());
    float yaw = std::atan2(wTz_c.rotation().coeff(1, 0), wTz_c.rotation().coeff(0, 0));
    // the Dubins steering function steers a car with its x-axis heading forward, so add pi/2
    _se2_state_a->setYaw(yaw + M_PI / 2.0f);
    _se2_state_b->setXY(wTz_t.translation().x(), wTz_t.translation().y());
    yaw = std::atan2(wTz_t.rotation().coeff(1, 0), wTz_t.rotation().coeff(0, 0));
    // the Dubins steering function steers a car with its x-axis heading forward, so add pi/2
    _se2_state_b->setYaw(yaw + M_PI / 2.0f);
    // compute dubins path
    bool first_time = false;
    _cache.pc.path = _dubins_state_space.dubins(_se2_state_a, _se2_state_b);
    _cache.path_computed = true;
    // compute the number of samples we need for each path segment
    Eigen::VectorXf wp(3);
    Eigen::VectorXf prev_wp(3);
    _cache.pc.num_samples[0] = 0;
    _cache.pc.num_samples[1] = 0;
    _cache.pc.num_samples[2] = 0;
    _cache.pc.t_offsets[0] = 0.0f;
    _cache.pc.t_offsets[1] = 0.0f;
    _cache.pc.t_offsets[2] = 0.0f;
    {
        float t = 0.0f;
        for (unsigned int s = 0; s < 3; ++s) {
            _cache.pc.t_offsets[s] = t;
            switch (_cache.pc.path.type_[s]) {
            case ::ompl::base::DubinsStateSpace::DubinsPathSegmentType::DUBINS_STRAIGHT: {
                _cache.pc.num_samples[s] = std::ceil(_cache.pc.path.length_[s] * _dubins_state_space.getTurningRadius() / _params.path_step_size);
                break;
            }
            case ::ompl::base::DubinsStateSpace::DubinsPathSegmentType::DUBINS_LEFT:
            case ::ompl::base::DubinsStateSpace::DubinsPathSegmentType::DUBINS_RIGHT: {
                // compute the state at the beginning and end of this segment
                sampleDubinsState(_cache.pc.path, t, first_time, prev_wp);
                sampleDubinsState(_cache.pc.path, t + _cache.pc.path.length_[s] / _cache.pc.path.length(), first_time, wp);
                // compute the center of rotation of the dubins path
                float beta = (M_PI - _cache.pc.path.length_[s]) / 2.0f;
                if (beta < 0.0f) {
                    beta = (-M_PI + _cache.pc.path.length_[s]) / 2.0f;
                }
                Eigen::Vector2f d = _dubins_state_space.getTurningRadius() * (wp.head(2) - prev_wp.head(2)).normalized();
                Eigen::Vector2f c = Eigen::Rotation2Df(-beta) * d + prev_wp.head(2);
                // compute robot state
                Eigen::Affine2f wTz = Eigen::Translation2f(prev_wp[0], prev_wp[1]) * Eigen::Rotation2Df(prev_wp[2] - M_PI / 2.0f);
                Eigen::Affine2f wTr = wTz * zTr;
                // compute radius on which robot moves around c
                float robot_r = (wTr.translation().head(2) - c).norm();
                _cache.pc.num_samples[s] = std::ceil(robot_r * _cache.pc.path.length_[s] / _params.path_step_size);
                break;
            }
            }
            t += _cache.pc.path.length_[s] / _cache.pc.path.length();
        }
    }
    // save transform zTr
    _cache.pc.zTr = zTr;
}

void QuasiStaticSE2Oracle::samplePath(ompl::control::TimedWaypoints* control) const
{
    // sample the path stored in _cache
    Eigen::VectorXf wp(3);
    Eigen::VectorXf prev_wp(3);
    float time_stamp = control->getDuration();
    // the given control is either empty, or contains an approach path to _cache.start_state
    if (control->numWaypoints() > 0) {
        prev_wp = std::get<1>(*(--(control->endWaypoints())));
        assert(prev_wp.head(3) == _cache.pc.start_state);
    } else {
        // first add start state
        control->addWaypoint(0.0f, _cache.pc.start_state);
        prev_wp = _cache.pc.start_state;
    }
    // run over path and sample as long as we haven't travelled more than _params.action_length
    // and haven't reached the end
    float robot_distance = 0.0f;
    bool first_time = false;
    while (robot_distance < _params.action_length and _cache.pc.nextSampleIdx()) {
        // sample _cache.sample_idx's robot state
        float t = _cache.pc.t_offsets[_cache.pc.seg_idx] + (float)(_cache.pc.in_seg_idx + 1) / (float)_cache.pc.num_samples[_cache.pc.seg_idx] * _cache.pc.path.length_[_cache.pc.seg_idx] / _cache.pc.path.length();
        mps_logging::logDebug(boost::format("Sampling t=%1%") % t, "[QuasiStaticSE2Oracle::samplePath]");
        sampleDubinsState(_cache.pc.path, t, first_time, wp);
        wp[2] -= M_PI / 2.0f;
        Eigen::Affine2f wTz = Eigen::Translation2f(wp[0], wp[1]) * Eigen::Rotation2Df(wp[2]);
        Eigen::Affine2f wTr = wTz * _cache.pc.zTr;
        wp[0] = wTr.translation().x();
        wp[1] = wTr.translation().y();
        wp[2] = std::atan2(wTr.rotation().coeff(1, 0), wTr.rotation().coeff(0, 0));
        float robot_delta = (prev_wp.head(2) - wp.head(2)).norm();
        robot_distance += robot_delta;
        time_stamp += std::max(robot_delta / _params.push_vel,
            std::abs(mps_math::shortest_direction_so2(prev_wp[2], wp[2])) / _params.rot_push_vel);
        // mps_logging::logDebug(boost::format("Adding waypoint (%1%: %2%, %3%, %4%)") % time_stamp % wp[0] % wp[1] % wp[2], log_prefix);
        control->addWaypoint(time_stamp, wp);
        prev_wp = wp;
    }
    // done sampling, update _cache
    _cache.pc.start_state = wp; // next start state is the last state added to this action
    // compute respective object state to detect failure in future iterations
    Eigen::Affine2f wTo = Eigen::Translation2f(wp[0], wp[1]) * Eigen::Rotation2Df(wp[2]) * _cache.pc.rTo;
    _cache.pc.obj_start_state[0] = wTo.translation().x();
    _cache.pc.obj_start_state[1] = wTo.translation().y();
    _cache.pc.obj_start_state[2] = std::atan2(wTo.rotation().coeff(1, 0), wTo.rotation().coeff(0, 0));
    if (_cache.pc.seg_idx == 3) { // we sampled the whole path, so reset the cache
        _cache.pc = PushingPathCache();
        _cache.path_computed = false;
    }

    // for (unsigned int s = 0; s < 3; ++s) {
    //     for (unsigned int i = 1; i <= num_samples[s]; ++i) {
    //         float t = t_offsets[s] + (float)i / (float)num_samples[s] * path.length_[s] / path.length();
    //         sampleDubinsState(path, t, first_time, wp);
    //         wp[2] -= M_PI / 2.0f;
    //         Eigen::Affine2f wTz = Eigen::Translation2f(wp[0], wp[1]) * Eigen::Rotation2Df(wp[2]);
    //         Eigen::Affine2f wTr = wTz * zTr;
    //         wp[0] = wTr.translation().x();
    //         wp[1] = wTr.translation().y();
    //         wp[2] = std::atan2(wTr.rotation().coeff(1, 0), wTr.rotation().coeff(0, 0));
    //         time_stamp += std::max((prev_wp.head(2) - wp.head(2)).norm() / _params.push_vel,
    //             std::abs(mps_math::shortest_direction_so2(prev_wp[2], wp[2])) / _params.rot_push_vel);
    //         // mps_logging::logDebug(boost::format("Adding waypoint (%1%: %2%, %3%, %4%)") % time_stamp % wp[0] % wp[1] % wp[2], log_prefix);
    //         control->addWaypoint(time_stamp, wp);
    //         // time_stamp += _params.path_step_size / _params.push_vel;
    //         prev_wp = wp;
    //     }
    // }
}

void QuasiStaticSE2Oracle::sampleDubinsState(::ompl::base::DubinsStateSpace::DubinsPath& path, float t, bool& first_time,
    Eigen::VectorXf& out) const
{
    static const std::string log_prefix("[QuasiStaticSE2Oracle::sampleDubinsState]");
    _dubins_state_space.interpolate(_se2_state_a, _se2_state_b, t, first_time, path, _se2_state_c);
    // mps_logging::logDebug(boost::format("z waypoint (%1%: %2%, %3%, %4%)") % t % _se2_state_c->getX() % _se2_state_c->getY() % _se2_state_c->getYaw(), log_prefix);
    out[0] = _se2_state_c->getX();
    out[1] = _se2_state_c->getY();
    out[2] = mps_math::normalize_orientation(_se2_state_c->getYaw());
}
/********************************** State generator helper ******************************************/
float QuasiStaticSE2Oracle::computeSamplingWeights(const ompl::state::SimEnvWorldState* current_state,
    const ompl::state::SimEnvWorldState* next_state, unsigned int obj_id,
    std::vector<float>& sampling_weights) const
{
    // compute pushing dir
    current_state->getObjectState(obj_id)->getConfiguration(_eigen_config);
    next_state->getObjectState(obj_id)->getConfiguration(_eigen_config2);
    Eigen::Vector2f dir = _eigen_config2.head(2) - _eigen_config.head(2);
    dir.normalize();
    Eigen::Rotation2D rot(_eigen_config[2]);
    // rate all pushing edge pairs
    float normalizer = 0.0f;
    auto& contact_pairs = _contact_pairs.at(obj_id);
    for (unsigned int pid = 0; pid < _contact_pairs.at(obj_id).size(); ++pid) {
        Eigen::Vector2f edge_normal = rot * contact_pairs.at(pid).object_edge->normal;
        sampling_weights.emplace_back(std::exp(-_params.exp_weight * (1.0f + dir.dot(edge_normal))));
        normalizer += sampling_weights.back();
    }
    return normalizer;
}

void QuasiStaticSE2Oracle::computeObjectRobotTransform(const PushingEdgePair& pair, float parallel_translation,
    float orthogonal_translation, Eigen::Affine2f& oTr) const
{
    Eigen::Vector2f pusher_pos = pair.object_edge->pcom + pair.object_edge->normal * orthogonal_translation + parallel_translation * pair.object_edge->dir;
    Eigen::Affine2f oTp = Eigen::Translation2f(pusher_pos) * Eigen::Rotation2D(pair.object_edge->pushing_angle);
    Eigen::Affine2f rTp = Eigen::Translation2f(pair.robot_edge->center) * Eigen::Rotation2Df(std::atan2(pair.robot_edge->normal[1], pair.robot_edge->normal[0]));
    Eigen::Affine2f pTr = rTp.inverse();
    oTr = oTp * pTr;
}

void QuasiStaticSE2Oracle::computeRobotState(Eigen::VectorXf& rob_state, const QuasiStaticSE2Oracle::PushingEdgePair& pair,
    const Eigen::VectorXf& obj_state, float translation, float orth_offset) const
{
    Eigen::Affine2f wTo = Eigen::Translation2f(obj_state.head(2)) * Eigen::Rotation2Df(obj_state[2]);
    Eigen::Affine2f oTr;
    computeObjectRobotTransform(pair, translation, orth_offset, oTr);
    Eigen::Affine2f wTr = wTo * oTr;
    rob_state.head(2) = wTr.translation();
    auto& rob_rot = wTr.rotation();
    rob_state[2] = std::atan2(rob_rot.coeff(1, 0), rob_rot.coeff(0, 0));
}

/************************************** Pre-processing Helper ************************************/
void QuasiStaticSE2Oracle::computeRobotPushingEdges()
{
    if (!_robot_edges.empty())
        return;
    auto robot = std::dynamic_pointer_cast<sim_env::Robot>(_objects.at(_robot_id));
    assert(robot);
    std::vector<sim_env::LinkPtr> links;
    robot->getLinks(links);
    for (auto link : links) {
        // TODO in case we have multiple links we would need to filter edges that only face other links
        std::vector<sim_env::Geometry> geometries(link->getGeometries());
        for (sim_env::Geometry& geom : geometries) {
            for (unsigned int vid = 1; vid < geom.vertices.size(); ++vid) {
                RobotPushingEdgePtr edge = std::make_shared<RobotPushingEdge>();
                edge->from = geom.vertices.at(vid - 1).head(2);
                edge->to = geom.vertices.at(vid).head(2);
                edge->center = (edge->from + edge->to) / 2.0f;
                edge->dir = edge->to - edge->from;
                edge->edge_length = edge->dir.norm();
                // check whether this edge is long enough
                if (edge->edge_length < 2.0 * _params.eps_min) {
                    continue;
                }
                edge->dir /= edge->edge_length;
                edge->normal[0] = edge->dir[1];
                edge->normal[1] = -edge->dir[0];
                _robot_edges.push_back(edge);
            }
        }
    }
}

typedef std::tuple<Eigen::Vector2f, Eigen::Vector2f, bool> Edge;
typedef bg::model::d2::point_xy<float> BoostPoint;
typedef bg::model::polygon<BoostPoint, false> BoostPolygon;
typedef bg::model::segment<BoostPoint> BoostSegment;

void extractEdges(BoostPolygon& geom, std::vector<Edge>& edges)
{
    // first add all edges of the original geometry and also store it in an rtree
    bg::index::rtree<BoostSegment, bg::index::linear<16>> tree;
    {
        auto& outer = geom.outer();
        for (unsigned int vid = 1; vid < outer.size(); ++vid) {
            tree.insert(BoostSegment(outer[vid - 1], outer[vid]));
            edges.emplace_back(std::make_tuple(Eigen::Vector2f(outer[vid - 1].get<0>(), outer[vid - 1].get<1>()),
                Eigen::Vector2f(outer[vid].get<0>(), outer[vid].get<1>()), false));
        }
    }
    // now run over the convex hull of the object and add the edges that aren't in rtree yet
    {
        BoostPolygon hull;
        bg::convex_hull(geom, hull);
        auto& outer = hull.outer();
        for (unsigned int vid = 1; vid < outer.size(); ++vid) {
            auto seg = BoostSegment(outer[vid - 1], outer[vid]);
            std::vector<BoostSegment> intersecting_edges;
            // tree.query(bg::index::nearest(seg, 1), std::back_inserter(nearest_edge));
            tree.query(bg::index::intersects(seg), std::back_inserter(intersecting_edges));
            // float edge_distance = bg::comparable_distance(seg, nearest_edge.back());
            // float edge_distance2 = bg::distance(seg, nearest_edge.back());
            bool normal_edge = false;
            for (auto& real_edge : intersecting_edges) {
                BoostSegment tseg(real_edge.first, real_edge.second);
                if (bg::equals(seg, tseg)) {
                    normal_edge = true;
                    break;
                }
            }
            // bool normal_edge = edge_distance == 0.0;
            if (!normal_edge) {
                edges.emplace_back(std::make_tuple(Eigen::Vector2f(outer[vid - 1].get<0>(), outer[vid - 1].get<1>()),
                    Eigen::Vector2f(outer[vid].get<0>(), outer[vid].get<1>()), true));
            }
        }
    }
}

void QuasiStaticSE2Oracle::computeObjectPushingEdges()
{
    if (!_obj_edges.empty())
        return;
    _obj_edges.resize(_objects.size());
    _contact_pairs.resize(_objects.size());
    // run over all objects
    for (unsigned int oid = 0; oid < _objects.size(); ++oid) {
        if (oid == _robot_id) { // skip robot
            continue;
        }
        auto obj = _objects.at(oid);
        // compute pushing edges for this object
        std::vector<sim_env::LinkPtr> links;
        obj->getLinks(links);
        // can only work with rigid bodies
        assert(links.size() == 1);
        Eigen::Vector3f com;
        sim_env::LinkPtr link = links.at(0);
        link->getLocalCenterOfMass(com);
        // get object geometry and compute convex hull
        std::vector<sim_env::Geometry> geometries(link->getGeometries());
        // TODO support multiple disjoint geometries?
        assert(geometries.size() == 1);
        // for this translate geometry into boost polygons
        BoostPolygon bp;
        for (auto& vertex : geometries.at(0).vertices) {
            bg::append(bp, BoostPoint(vertex[0], vertex[1]));
        }
        std::vector<Edge> edges;
        extractEdges(bp, edges);
        // now run over all edges and extract pushing edges
        for (auto& edge : edges) {
            ObjectPushingEdgePtr pedge = std::make_shared<ObjectPushingEdge>();
            pedge->from = std::get<0>(edge);
            pedge->to = std::get<1>(edge);
            pedge->dir = pedge->to - pedge->from;
            pedge->edge_length = pedge->dir.norm();
            // check whether this edge is long enough
            if (pedge->edge_length < 2.0f * _params.eps_min) {
                continue;
            }
            pedge->dir /= pedge->edge_length;
            pedge->normal[0] = pedge->dir[1];
            pedge->normal[1] = -pedge->dir[0];
            pedge->pushing_angle = std::atan2(-pedge->normal[1], -pedge->normal[0]);
            pedge->edge_angle = std::atan2(pedge->dir[1], pedge->dir[0]);
            // test whether this edge is suitable for stable pushing
            Eigen::Vector2f rel_com = com.head(2) - pedge->from;
            rel_com = rel_com - rel_com.dot(pedge->normal) * pedge->normal;
            float sdist = rel_com.dot(pedge->dir);
            pedge->pcom = rel_com + pedge->from;
            // check whether the projected center of mass falls onto this edge
            if (sdist < _params.eps_min or sdist > pedge->edge_length - _params.eps_min) {
                continue;
            }
            _obj_edges.at(oid).push_back(pedge);
            // compute what robot edge can be used to push this edge
            float min_edge_length = 2.0f * _params.eps_min;
            if (std::get<2>(edge)) { // object edge is from convex hull, i.e. only end points are actually part of the object
                min_edge_length = pedge->edge_length;
            }
            for (auto& robot_edge : _robot_edges) {
                if (robot_edge->edge_length > min_edge_length) {
                    PushingEdgePair pepair;
                    pepair.robot_edge = robot_edge;
                    pepair.object_edge = pedge;
                    pepair.max_translation = 0.0f;
                    pepair.min_translation = 0.0f;
                    if (std::get<2>(edge)) {
                        pepair.min_translation = (pedge->to - pedge->pcom).norm() - robot_edge->edge_length / 2.0f;
                        pepair.max_translation = robot_edge->edge_length / 2.0f - (pedge->from - pedge->pcom).norm();
                    } else {
                        pepair.min_translation = -std::max(robot_edge->edge_length / 2.0f - _params.eps_min, 0.0f);
                        pepair.max_translation = std::max(robot_edge->edge_length / 2.0f - _params.eps_min, 0.0f);
                    }
                    // bool valid = true;
                    bool valid = computeCollisionFreeRange(pepair, oid);
                    if (valid)
                        _contact_pairs.at(oid).push_back(pepair);
                }
            }
        }
        if (_contact_pairs.at(oid).empty()) {
            throw std::runtime_error("Failed to compute any pushing edge pairs for object " + _objects.at(oid)->getName());
        }
    }
}

bool QuasiStaticSE2Oracle::computeCollisionFreeRange(QuasiStaticSE2Oracle::PushingEdgePair& pair, unsigned int oid)
{
    assert(_objects.at(oid)->getNumActiveDOFs() == 3); // this only works for planar rigid bodies
    auto robot = _objects.at(_robot_id);
    _eigen_config.resize(3);
    assert(robot->getNumActiveDOFs() == 3); // this only works for planar robots
    auto obj_state(_objects.at(oid)->getDOFPositions());
    // init variables for algorithm
    bool col_free_exists = false; // whether there is any collision-free state
    auto best_interval = std::make_pair(pair.min_translation, pair.min_translation); // best interval of translations
    float best_interval_length = 0.0f; // length of that interval
    auto current_interval = std::make_pair(pair.min_translation, pair.min_translation); // current interval under investigation
    bool current_interval_valid = false; // whether we currently have a valid interval growing

    // sample range [pair.min_translation, pair.max_translation] to find the largest collision-free subinterval
    float current_t = pair.min_translation;
    while (current_t <= pair.max_translation) {
        // check collision for current_t
        computeRobotState(_eigen_config, pair, obj_state, current_t, _params.eps_dist);
        robot->setDOFPositions(_eigen_config);
        if (robot->checkCollision(_objects.at(oid))) { // does it collide?
            // if the current interval is valid, close it
            if (current_interval_valid) {
                float interval_length = current_interval.second - current_interval.first;
                // save it if it's longer than the best one so far
                if (interval_length >= best_interval_length) {
                    best_interval_length = interval_length;
                    best_interval = current_interval;
                }
                current_interval_valid = false;
            }
        } else {
            col_free_exists = true;
            if (!current_interval_valid) {
                // start new interval
                current_interval.first = current_t;
                current_interval_valid = true;
            }
            // grow interval
            current_interval.second = current_t;
        }
        current_t += _params.col_sample_step;
    }
    // close the last current_interval in case it is valid
    if (current_interval_valid) {
        float interval_length = current_interval.second - current_interval.first;
        if (interval_length >= best_interval_length) {
            best_interval_length = interval_length;
            best_interval = current_interval;
        }
    }
    // save the best interval
    pair.min_translation = best_interval.first;
    pair.max_translation = best_interval.second;
    assert(pair.min_translation <= pair.max_translation);
    return col_free_exists;
}