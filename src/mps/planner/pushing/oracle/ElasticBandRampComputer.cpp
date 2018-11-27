#include <mps/planner/pushing/oracle/ElasticBandRampComputer.h>
#include <sim_env/SimEnv.h>
#include <sim_env/Grid.h>
// #include <callgrind.h>

using namespace mps::planner::pushing::oracle;
#define NUMERICAL_EPSILON 0.00000001f

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////// ElasticBandRampComputer::Parameters ////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ElasticBandRampComputer::Parameters::Parameters() {
    step_size = 0.02f;
    multiplier_rot_gradient = 1.0f;
    min_euclidean_dist = 0.025f;
    min_radian_dist = 0.08f;
    safety_margin = 0.1f;
    gamma = 1.5f;
    oscillation_threshold = 0.9f;
    iterations_multiplier = 3.0f;
    look_ahead_weight = 0.5f;
}

ElasticBandRampComputer::Parameters::Parameters(const ElasticBandRampComputer::Parameters& other) = default;

ElasticBandRampComputer::Parameters::~Parameters() = default;

ElasticBandRampComputer::Parameters& ElasticBandRampComputer::Parameters::operator=(const ElasticBandRampComputer::Parameters& other) = default;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////// ElasticBandRampComputer ///////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ElasticBandRampComputer::ElasticBandRampComputer(const Parameters& params) : _parameters(params) {
}

ElasticBandRampComputer::~ElasticBandRampComputer() = default;

void ElasticBandRampComputer::steer(const Eigen::VectorXf &current_robot_state,
                                    const Eigen::VectorXf &desired_robot_state,
                                    std::vector<Eigen::VectorXf> &control_params) const
{
    static const std::string log_prefix("[mps::planner::pushing::oracle::ElasticBandRampComputer::steer]");
    if (!_world) {
        throw std::logic_error(log_prefix + " steer(..) called before this instance was intialized.");
    }
    // If we do not know the world state, we can not use the sdf, so just call the normal ramp computer
    _ramp_computer->steer(current_robot_state, desired_robot_state, control_params);
}

void ElasticBandRampComputer::steer(const ompl::state::SimEnvObjectState* current_robot_state,
                                    const ompl::state::SimEnvObjectState* desired_robot_state,
                                    std::vector<::ompl::control::Control*> &controls) const
{
    static const std::string log_prefix("[mps::planner::pushing::oracle::ElasticBandRampComputer::steer]");
    if (!_world) {
        throw std::logic_error(log_prefix + " steer(..) called before this instance was intialized.");
    }
    // If we do not know the world state, we can not use the sdf,
    // so just call the normal ramp computer
    _ramp_computer->steer(current_robot_state, desired_robot_state, controls);
}

void mps::planner::pushing::oracle::ElasticBandRampComputer::steer(
                                    const ompl::state::SimEnvObjectState* current_robot_state,
                                    const ompl::state::SimEnvObjectState* desired_robot_state,
                                    const ompl::state::SimEnvWorldState* current_world_state,
                                    std::vector<Eigen::VectorXf>& control_params) const
{
    // CALLGRIND_TOGGLE_COLLECT;
    static const std::string log_prefix("[mps::planner::pushing::oracle::ElasticBandRampComputer::steer(with world state)]");
    if (!_world) {
        throw std::logic_error(log_prefix + " steer(..) called before this instance was intialized.");
    }
    auto logger = _world->getLogger();
    logger->logDebug("Juhu I'm being asked to do sth with my SDF!", log_prefix);
    // first compute a path
    std::list<Eigen::VectorXf> path;
    computePath(current_robot_state, desired_robot_state, current_world_state, path);
    assert(path.size() >= 2); // should always contain start and end state
    // next compute ramps between each path waypoint
    auto path_iter = path.begin();
    auto& previous_config = *path_iter;
    ++path_iter;
    while (path_iter != path.end()) {
        auto& current_config = *path_iter;
        _ramp_computer->steer(previous_config, current_config, control_params);
        previous_config = current_config;
        ++path_iter;
    }
    // CALLGRIND_TOGGLE_COLLECT;
}

void ElasticBandRampComputer::init(sim_env::WorldPtr world,
                                   sim_env::RobotPtr robot,
                                   ompl::state::SimEnvWorldStateSpacePtr world_state_space,
                                   ompl::control::RampVelocityControlSpacePtr control_space,
                                   const ompl::state::PlanningSceneBounds& bounds,
                                   float sdf_resolution, float error_threshold, bool force_new_sdf)
{

    static const std::string log_prefix("[mps::planner::pushing::oracle::ElasticBandRampComputer::init]");
    bool b_update_sdf = force_new_sdf;
    sim_env::BoundingBox world_bounds;
    float robot_padding = robot->getLocalAABB().extents().norm();
    world_bounds.min_corner[0] = bounds.x_limits[0] - 2 * robot_padding;
    world_bounds.min_corner[1] = bounds.y_limits[0] - 2 * robot_padding;
    // TODO what to do about this? we certainly don't wanna make this larger in the 2d case
    world_bounds.min_corner[2] = bounds.z_limits[0];
    world_bounds.max_corner[0] = bounds.x_limits[1] + 2 * robot_padding;
    world_bounds.max_corner[1] = bounds.y_limits[1] + 2 * robot_padding;
    // TODO what to do about this? we certainly don't wanna make this larger in the 2d case
    world_bounds.max_corner[2] = bounds.z_limits[1];
    if (!_world) { // first intialization
        _world = world;
        _robot = robot;
        _world_state_space = world_state_space;
        auto robot_space = world_state_space->getObjectStateSpace(robot->getName());
        auto robot_config_space = robot_space->getConfigurationSpace();
        _gradient_deltas = robot_config_space->getGradientDeltas();
        _scene_sdf = std::make_shared<mps::sdf::SceneSDF>(_world);
        _ramp_computer = std::unique_ptr<oracle::RampComputer>(new oracle::RampComputer(robot_config_space, control_space));
        b_update_sdf = true;
    } else {
        // b_update_sdf |= robot_space->getName() != _robot_space->getName();
        b_update_sdf |= _robot != robot;
        _robot = robot;
        _world_state_space = world_state_space;
        auto robot_space = world_state_space->getObjectStateSpace(robot->getName());
        auto robot_config_space = robot_space->getConfigurationSpace();
        _gradient_deltas = robot_config_space->getGradientDeltas();
        _ramp_computer = std::unique_ptr<oracle::RampComputer>(new oracle::RampComputer(robot_config_space, control_space));
        // check whether the sim_env world is different and we need to create a new sdf
        sdf::SceneSDF::SceneSDFConstructionState would_be_state(world, world_bounds);
        b_update_sdf |= would_be_state.isDifferent(_sdf_construction_state);
    }
    if (_robot->getNumActiveDOFs() != 3) {
        throw std::logic_error("[ElasticBandRampComputer] This class can only operate on planar holonomic robots with 3 DOFs x,y,theta");
    }

    if (b_update_sdf) { // finally create sdf if need to
        // safety guard
        if (sdf_resolution <= 0.0f) {
            throw std::runtime_error("[ElasticBandRampComputer] SDF resolution needs to be positive, but non-positive number given.");
        }
        if (error_threshold <= 0.0f or error_threshold > 1.0f) {
            throw std::runtime_error("[ElasticBandRampComputer] Error threshold needs to be in interval (0, 1]");
        }
        // TODO we could/should load the sdf here from file if available
        _world->getLogger()->logDebug("Computing signed distance field for world", log_prefix);
        _sdf_construction_state = _scene_sdf->computeSDF(world_bounds, sdf_resolution, sdf_resolution, error_threshold);
        _world->getLogger()->logDebug("SDF computation finished", log_prefix);
    }
}

EBDebugDrawerPtr ElasticBandRampComputer::getDebugDrawer() const {
    static const std::string log_prefix("[mps::planner::pushing::oracle::ElasticBandRampComputer::getDebugDrawer]");
    if (!_world) {
        throw std::logic_error(log_prefix + "getDebugDrawer called before this instance was intialized.");
    }
    if (!_debug_drawer) {
        _debug_drawer = std::make_shared<EBDebugDrawer>(_world->getViewer());
    }
    return _debug_drawer;
}

void ElasticBandRampComputer::renderSDF(float res) {
    static const std::string log_prefix("[mps::planner::pushing::oracle::ElasticBandRampComputer::renderSDF]");
    if (!_world) {
        throw std::logic_error(log_prefix + "renderSDF called before this instance was intialized.");
    }
    auto debug_drawer = getDebugDrawer();
    debug_drawer->renderSDF(_scene_sdf, res);
}

void ElasticBandRampComputer::computePath(const ompl::state::SimEnvObjectState* start_state,
                                          const ompl::state::SimEnvObjectState* goal_state,
                                          const ompl::state::SimEnvWorldState* world_state,
                                          std::list<Eigen::VectorXf>& waypoints) const
{
    static const std::string log_prefix("[mps::planner::pushing::oracle::ElasticBandRampComputer::computePath]");
    auto logger = _world->getLogger();
    // auto debug_drawer = getDebugDrawer();
    // debug_drawer->clear();
    // try something really simple, but fast: potential field approach
    // first set the world to the start state and update sdf
    _world->saveState();
    _world_state_space->setToState(_world, world_state);
    _scene_sdf->updateTransforms();
    // initialize start and end point
    Eigen::VectorXf current_point;
    start_state->getConfiguration(current_point);
    Eigen::VectorXf end_point;
    goal_state->getConfiguration(end_point);
    // {
    //     using namespace std::placeholders;
    //     auto potential_fn = std::bind(&ElasticBandRampComputer::computePotential, this, _1, end_point);
    //     debug_drawer->renderPotential(_scene_sdf->getBoundingBox(), 0.02f, 0.0f, potential_fn);
    // }
    waypoints.push_back(current_point);
    float current_distance = (current_point - end_point).norm();
    unsigned int min_num_samples = std::floor(current_distance / _parameters.step_size);
    unsigned int max_num_iterations = std::floor(_parameters.iterations_multiplier * min_num_samples);
    unsigned num_iterations = 0;
    Eigen::VectorXf gradient(current_point.size());
    Eigen::VectorXf prev_gradient(current_point.size());
    prev_gradient.setZero();
    Eigen::VectorXf look_ahead_gradient(prev_gradient);
    Eigen::VectorXf look_ahead_point(prev_gradient);
    // std::vector<float> distances;
    // distances.reserve(max_num_iterations);
    while (((current_point.head<2>() - end_point.head<2>()).norm() > _parameters.step_size)
            and num_iterations < max_num_iterations) {
        // compute gradient
        float distance = computeGradient(current_point, end_point, gradient);
        // we add the previous waypoint if it is close to obstacles
        if (distance < _parameters.safety_margin) {
            // we potentially have a gradient that moves us away from obstacles, so save this waypoint
            waypoints.push_back(current_point);
        }
        // if the gradient diminishes, we are in a local optimum
        float gradient_norm = gradient.head<2>().norm();
        if (gradient_norm + gradient[2] <= NUMERICAL_EPSILON) {
            logger->logDebug("Ran into a local optimum - gradient is zero. Can not proceed with potential field descent",
                             log_prefix);
            break;
        }
        // compute Nesterov accelerated gradient
        look_ahead_point.head<2>() = current_point.head<2>() - _parameters.step_size * 1.0f / gradient_norm * gradient.head<2>();
        look_ahead_point[2] = current_point[2] - _parameters.multiplier_rot_gradient * gradient[2];
        computeGradient(look_ahead_point, end_point, look_ahead_gradient);
        logger->logDebug(boost::format("Gradient is %1% and look-ahead-gradient is %2%") % gradient.transpose() % look_ahead_gradient.transpose(), log_prefix);
        gradient = (1.0f - _parameters.look_ahead_weight) * gradient + _parameters.look_ahead_weight * look_ahead_gradient;
        logger->logDebug(boost::format("Merged gradient is %1%") % gradient.transpose(), log_prefix);
        gradient_norm = gradient.head<2>().norm();
        // check whether we are oscillating
        if (gradient.dot(-prev_gradient) > _parameters.oscillation_threshold) {
            logger->logDebug("We seem to be oscillating, aborting gradient descent.",
                             log_prefix);
            break;
        }
        // now we are not in a local optima (or at least we do not know)
        current_point.head<2>() -= _parameters.step_size * 1.0f / gradient_norm * gradient.head<2>();
        current_point[2] -= _parameters.multiplier_rot_gradient * gradient[2];
        prev_gradient = gradient;
        ++num_iterations;
        // debug_drawer->clear();
        // debug_drawer->drawPath(waypoints);
    }
    // before we return this path, let's try to simplify the path a little bit more
    removeRepetitions(waypoints); // removes waypoints that are too similiar to previously visited waypoints (oscillation reduction)
    // in any case push back the end point, the higher level logic wil figure out that the last action might be invalid
    waypoints.push_back(end_point);
    simplifyPath(waypoints); // removes waypoints that lie on a straight line
    _world->restoreState();
    // debug_drawer->drawPath(waypoints);
}

float ElasticBandRampComputer::computeGoalPotential(const Eigen::VectorXf& robot_config, const Eigen::VectorXf& goal_config) const {
    return 0.5f * (robot_config - goal_config).norm();
}

void ElasticBandRampComputer::computeGoalGradient(const Eigen::VectorXf& robot_config,
                                                  const Eigen::VectorXf& goal,
                                                  Eigen::VectorXf& gradient) const {
    gradient = robot_config - goal;
}

float ElasticBandRampComputer::computeObstaclePotential(const Eigen::VectorXf& robot_config) const {
    float dist = computeObstacleDistance(robot_config);
    if (dist >= _parameters.safety_margin) {
        return 0.0f;
    }
    if (dist >= 0.0f) {
        // return _parameters.gamma * (dist - _parameters.safety_margin) * (dist - _parameters.safety_margin) / (_parameters.safety_margin * _parameters.safety_margin);
        return _parameters.gamma / (2.0f * _parameters.safety_margin) *
                (dist - _parameters.safety_margin)* (dist - _parameters.safety_margin); // cost function from CHOMP paper
    }
    return _parameters.gamma * (_parameters.safety_margin / 2.0f - dist); // cost function from CHOMP paper
}

float ElasticBandRampComputer::computeObstacleGradient(const Eigen::VectorXf& robot_config,
                                                       Eigen::VectorXf& gradient) const {
    float dist = computeObstacleDistance(robot_config);
    if (dist > _parameters.safety_margin) { // gradient = 0
        gradient.resize(robot_config.size());
        gradient.setZero();
    } else if (dist >= 0.0f) { // gradient = gamma / epsilon * (d(q) - epsilon) * grad(d(q))
        computeObstacleDistanceGradient(robot_config, gradient);
        gradient = _parameters.gamma / _parameters.safety_margin * (dist - _parameters.safety_margin) * gradient;
    } else { // gradient = -gamma grad(d(q))
        computeObstacleDistanceGradient(robot_config, gradient);
        gradient = -_parameters.gamma * gradient;
    }
    return dist;
    // gradient = _parameters.gamma * 2.0f / (_parameters.safety_margin * _parameters.safety_margin) * (dist - _parameters.safety_margin) * gradient;
}

float ElasticBandRampComputer::computePotential(const Eigen::VectorXf& config, const Eigen::VectorXf& goal) const {
    return computeGoalPotential(config, goal) + computeObstaclePotential(config);
}

float ElasticBandRampComputer::computeGradient(const Eigen::VectorXf& config,
                                              const Eigen::VectorXf& goal,
                                              Eigen::VectorXf& gradient) const {
    Eigen::VectorXf goal_gradient(config.size());
    Eigen::VectorXf obstacle_gradient(config.size());
    computeGoalGradient(config, goal, goal_gradient);
    float distance = computeObstacleGradient(config, obstacle_gradient);
    // we don't want our attraction force to scale with the distance to our goal, we prefer constant speed as this allows us
    // to balance between obstacle_gradient and goal approach better
    goal_gradient.normalize();
    gradient = goal_gradient + obstacle_gradient;
    auto logger = _world->getLogger();
    logger->logDebug(boost::format("For configurations %1%") % config.transpose(), "computeGradient");
    logger->logDebug(boost::format("Goal gradient is %1% and obstacle gradient is %2%") % goal_gradient.transpose() % obstacle_gradient.transpose(), "computeGradient");
    return distance;
}

float ElasticBandRampComputer::computeObstacleDistance(const Eigen::VectorXf& config) const {
    static const std::string log_prefix("[mps::planner::pushing::oracle::ElasticBandRampComputer::computeObstacleDistance]");
    auto logger = _world->getLogger();
    _robot->setDOFPositions(config);
    // get ball approximation for robot in this configuration
   _robot->getBallApproximation(_balls);
    float min_distance = std::numeric_limits<float>::max();
    for (auto& ball : _balls) {
        // for each ball compute how far it is from penetrating an obstacle
        float distance = _scene_sdf->getDistanceDirty(ball.center) - ball.radius;
        min_distance = std::min(min_distance, distance);
    }
//    logger->logDebug(boost::format("Minimal distance of config %1% is %2%") % config.transpose() % min_distance, log_prefix);
    return min_distance;
}

void ElasticBandRampComputer::computeObstacleDistanceGradient(const Eigen::VectorXf& robot_config,
                                                               Eigen::VectorXf& gradient) const
{
    // TODO we should save sdf gradients and compute this gradient using jacobians for each ball instead
    // static const std::string log_prefix("[ElasticBandRampComputer::computeCollisionGradient]");
    // for now numerical gradient computation
    Eigen::VectorXf query_config(robot_config);
    gradient.resize(robot_config.size());
    // compute gradient for robot in configuration robot_config
    for (unsigned int dof = 0; dof < robot_config.size(); ++dof) {
        query_config[dof] = robot_config[dof] + _gradient_deltas[dof];
        float fn_delta_plus = computeObstacleDistance(query_config);
        query_config[dof] = robot_config[dof] - _gradient_deltas[dof];
        float fn_delta_minus = computeObstacleDistance(query_config);
        gradient[dof] = (fn_delta_plus - fn_delta_minus) / (2.0f * _gradient_deltas[dof]);
        query_config[dof] = robot_config[dof];
    }
}

void ElasticBandRampComputer::removeRepetitions(std::list<Eigen::VectorXf>& waypoints) const {
    static const std::string log_prefix("[ElasticBandRampComputer::removeRepetitions]");
    auto logger = _world->getLogger();
    // first check whether it makes sense to remove anything
    if (waypoints.size() < 2) return; // if we have at least 2 points, then we might be able to remove something
    std::vector<Eigen::VectorXf> prev_waypoints;
    auto wp_iter = waypoints.begin();
    prev_waypoints.push_back(*wp_iter);
    ++wp_iter;
    prev_waypoints.push_back(*wp_iter);
    ++wp_iter;
    while (wp_iter != waypoints.end()) {
        bool delete_wp = false;
        Eigen::VectorXf& wp = *wp_iter;
        // check whether we've been at this configuration before
        for (auto& prev_wp : prev_waypoints) {
            if (((wp.head<2>() - prev_wp.head<2>()).norm() < _parameters.min_euclidean_dist)
                and (std::abs(wp[2] - prev_wp[2]) < _parameters.min_radian_dist)) {
                // if yes, remove this waypoint because this just means we are oscillating here
                delete_wp = true;
                break;
            }
        }
        if (delete_wp) {
            logger->logDebug(boost::format("We seem to be oscillating at waypoint %1%, removing it.") % wp.transpose(),
                            log_prefix);
            wp_iter = waypoints.erase(wp_iter);
        } else {
            prev_waypoints.push_back(wp);
            ++wp_iter;
        }
    }
}

void ElasticBandRampComputer::simplifyPath(std::list<Eigen::VectorXf>& waypoints) const {
    static const std::string log_prefix("[ElasticBandRampComputer::simplifyPath]");
    auto logger = _world->getLogger();
    // first check whether it makes sense to remove anything
    if (waypoints.size() < 3) return; // we need at least 3 points for this
    Eigen::Vector2f travel_dir;
    Eigen::Vector2f local_dir;
    Eigen::Vector2f normal;
    Eigen::VectorXf pre_prev_wp;
    Eigen::VectorXf prev_wp;
    auto wp_iter = waypoints.begin();
    pre_prev_wp = *wp_iter;
    ++wp_iter;
    auto prev_wp_iter = wp_iter;
    prev_wp = *wp_iter;
    ++wp_iter;
    while (wp_iter != waypoints.end()) {
        bool remove_waypoint = false;
        Eigen::VectorXf& current_wp = *wp_iter;
        travel_dir = current_wp.head<2>() - pre_prev_wp.head<2>();
        local_dir = prev_wp.head<2>() - pre_prev_wp.head<2>();
        float travel_distance = travel_dir.norm();
        // there is the posibility that we did not travel in position but only in orientation
        if (travel_distance > NUMERICAL_EPSILON) { // if pre_prev_wp and current_wp are at different locations
            travel_dir *= 1.0f / travel_distance;
            computeNormal(travel_dir, normal); // compute the normal of the travel direction
            float distance = std::abs(local_dir.dot(normal)); // how far is prev_wp away from that straight line movement?
            // decide whether this is close enough to remove
            if (distance <= _parameters.min_euclidean_dist) {
                // if yes, we need to check whether this waypoint actually lies within the line segment pre_prev_wp -> current_wp
                Eigen::Vector2f projected_wp = prev_wp.head<2>() - distance * normal;
                float t = travel_dir.dot(projected_wp - pre_prev_wp.head<2>());
                if (t > 0 and t < travel_distance) { // if lies on the line segment
                    // we need to check whether the angle is similar to what it would be if we remove the waypoint
                    float normalized_t = t / travel_distance;
                    float would_be_angle = normalized_t * current_wp[2] + (1.0f - normalized_t) * pre_prev_wp[2];
                    remove_waypoint = std::abs(would_be_angle - prev_wp[2]) <= _parameters.min_radian_dist;
                }
            }
        } else { // we are at the same location in pre_pre_wp and current_wp, make the decision on how far away prev_wp is
            remove_waypoint = local_dir.norm() <= _parameters.min_euclidean_dist and
                              std::abs(prev_wp[2] - current_wp[2]) <= _parameters.min_radian_dist;
        }
        if (remove_waypoint) {
            // erase previous waypoint
            prev_wp_iter = waypoints.erase(prev_wp_iter); // prev_wp_iter = wp_iter
            prev_wp = current_wp; // update previous waypoint to be current_wp
            ++wp_iter; // proceed with wp_iter
        } else { // no removal
            pre_prev_wp = prev_wp; // everything moves one step further
            prev_wp = current_wp;
            ++prev_wp_iter;
            ++wp_iter;
        }
    }
}

void ElasticBandRampComputer::computeNormal(const Eigen::Vector2f& dir, Eigen::Vector2f& normal) const {
    if (std::abs(dir[0]) <= NUMERICAL_EPSILON) {
        normal[0] = 1.0f;
        normal[1] = 0.0f;
    } else {
        normal[0] = -dir[1] / dir[0];
        normal[1] = 1.0f;
        normal.normalize();
    }
}
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////// EBDebugDrawer /////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////
EBDebugDrawer::EBDebugDrawer(sim_env::WorldViewerPtr viewer) : _world_viewer(viewer), _sdf_drawing_handle(false)
{
}

EBDebugDrawer::~EBDebugDrawer() = default;

void EBDebugDrawer::renderSDF(mps::sdf::SceneSDFPtr sdf, float resolution) {
    static const std::string log_prefix("[mps::planner::pushing::algorithm::DebugDrawer::renderSDF]");
    _world_viewer->getWorld()->getLogger()->logDebug("Rendering scene sdf", log_prefix);
    auto aabb = sdf->getBoundingBox();
    sim_env::grid::VoxelGrid<float, Eigen::Vector4f> color_grid(aabb.min_corner, aabb.max_corner, resolution);
    float min_distance = std::numeric_limits<float>::max();
    // fix the maximal distance to the size of the grid, for more consistent rendering
    float max_distance = aabb.extents().norm();
    sdf->updateTransforms();
    {
        _world_viewer->getWorld()->getLogger()->logDebug("Sampling distances", log_prefix);
        auto idx_gen = color_grid.getIndexGenerator();
        // first query all distances
        while (idx_gen.hasNext()) {
            auto idx = idx_gen.next();
            Eigen::Vector3f voxel_pos;
            color_grid.getCellPosition(idx, voxel_pos, true);
            voxel_pos[2] = 0.0f; // TODO this is a hack for 2d grids where our rendering grid has a lower resolution than our sdf
            float distance = sdf->getDistanceDirty(voxel_pos);
            // max_distance = std::max(distance, max_distance);
            min_distance = std::min(distance, min_distance);
            color_grid(idx)[0] = distance;
            color_grid(idx)[1] = 0.0f;
            color_grid(idx)[2] = 0.0f;
            color_grid(idx)[3] = 0.0f;
        }
    }
    // next normalize colors based on distances
    Eigen::Vector4f min_color(1.0f, 0.0f, 0.0f, 1.0f);
    Eigen::Vector4f zero_color(1.0f, 1.0f, 0.0f, 1.0f);
    Eigen::Vector4f max_color(0.0f, 0.0f, 1.0f, 1.0f);
    {
        _world_viewer->getWorld()->getLogger()->logDebug("Converting distances to colors", log_prefix);
        auto idx_gen = color_grid.getIndexGenerator();
        while (idx_gen.hasNext()) {
            auto idx = idx_gen.next();
            float distance = color_grid(idx)[0];
            // color_grid(idx) = (distance - min_distance) / normalizer * (max_color - min_color) + min_color;
            if (distance < 0.0f) {
                color_grid(idx) = distance / min_distance * (min_color - zero_color) + zero_color;
            } else {
                color_grid(idx) = std::min(distance / max_distance, 1.0f) * (max_color - zero_color) + zero_color;
            }
        }
    }
    _world_viewer->getWorld()->getLogger()->logDebug("Done, forwarding color grid to sim_env viewer", log_prefix);
    _sdf_drawing_handle = _world_viewer->drawVoxelGrid(color_grid, _sdf_drawing_handle);
}

void EBDebugDrawer::renderPotential(const sim_env::BoundingBox& aabb, float resolution, float robot_orientation,
                                    PotentialFunction potential_fn) {
    sim_env::grid::VoxelGrid<float, Eigen::Vector4f> color_grid(aabb.min_corner, aabb.max_corner, resolution);
    float min_value = std::numeric_limits<float>::max();
    float max_value = std::numeric_limits<float>::lowest();
    {
        auto idx_gen = color_grid.getIndexGenerator();
        Eigen::VectorXf query_config(3);
        query_config[2] = robot_orientation;
        // query all values
        while (idx_gen.hasNext()) {
            auto idx = idx_gen.next();
            Eigen::Vector3f voxel_pos;
            color_grid.getCellPosition(idx, voxel_pos, true);
            query_config[0] = voxel_pos[0];
            query_config[1] = voxel_pos[1];
            float value = potential_fn(query_config);
            max_value = std::max(value, max_value);
            min_value = std::min(value, min_value);
            color_grid(idx)[0] = value;
            color_grid(idx)[1] = 0.0f;
            color_grid(idx)[2] = 0.0f;
            color_grid(idx)[3] = 0.0f;
        }
    }
    // next normalize colors based on values
    Eigen::Vector4f min_color(0.0f, 0.0f, 1.0f, 1.0f);
    Eigen::Vector4f max_color(1.0f, 0.0f, 0.0f, 1.0f);
    {
        auto idx_gen = color_grid.getIndexGenerator();
        while (idx_gen.hasNext()) {
            auto idx = idx_gen.next();
            float value = color_grid(idx)[0];
            color_grid(idx) = (value - min_value) / (max_value - min_value) * (max_color - min_color) + min_color;
        }
    }
    _handles.push_back(_world_viewer->drawVoxelGrid(color_grid));
}

void EBDebugDrawer::drawBalls(std::vector<sim_env::Ball>& balls) {
    Eigen::Vector4f color(0.0, 0.0, 0.0, 0.5);
    for (auto& ball : balls) {
        _handles.push_back(_world_viewer->drawSphere(ball.center, ball.radius, color));
    }
}

void EBDebugDrawer::drawPath(std::list<Eigen::VectorXf>& waypoints) {
    for (auto& wp : waypoints) {
        Eigen::Vector3f center(wp[0], wp[1], 0.0f);
        _handles.push_back(_world_viewer->drawSphere(wp, 0.02f, Eigen::Vector4f(0.0f, 0.0f, 0.0f, 1.0f), 0.01f));
    }
}

void EBDebugDrawer::clearSDFRendering() {
    if (_sdf_drawing_handle.isValid()) {
        _world_viewer->removeDrawing(_sdf_drawing_handle);
        _sdf_drawing_handle = sim_env::WorldViewer::Handle(false);
    }
}

void EBDebugDrawer::clear() {
    for (auto& handle : _handles) {
        _world_viewer->removeDrawing(handle);
    }
    _handles.clear();
    clearSDFRendering();
}

