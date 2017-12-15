#include <mps/planner/pushing/oracle/ElasticBandRampComputer.h>
#include <sim_env/SimEnv.h>
#include <sim_env/Grid.h>

using namespace mps::planner::pushing::oracle;

ElasticBandRampComputer::ElasticBandRampComputer() = default;

ElasticBandRampComputer::~ElasticBandRampComputer() = default;

void ElasticBandRampComputer::steer(const Eigen::VectorXf &current_robot_state,
                                    const Eigen::VectorXf &desired_robot_state,
                                    std::vector<Eigen::VectorXf> &control_params) const
{
    static const std::string log_prefix("[mps::planner::pushing::oracle::ElasticBandRampComputer::steer]");
    if (!_world) {
        throw std::logic_error(log_prefix + "steer called before this instance was intialized.");
    }
    // TODO
}

void ElasticBandRampComputer::init(sim_env::WorldPtr world,
                                   ompl::state::SimEnvObjectConfigurationSpacePtr robot_space,
                                   ompl::control::RampVelocityControlSpacePtr control_space,
                                   const ompl::state::PlanningSceneBounds& bounds,
                                   float sdf_resolution, float error_threshold, bool force_new_sdf)
{

    static const std::string log_prefix("[mps::planner::pushing::oracle::ElasticBandRampComputer::init]");
    bool b_update_sdf = force_new_sdf;
    sim_env::BoundingBox world_bounds;
    world_bounds.min_corner[0] = bounds.x_limits[0];
    world_bounds.min_corner[1] = bounds.y_limits[0];
    world_bounds.min_corner[2] = bounds.z_limits[0];
    world_bounds.max_corner[0] = bounds.x_limits[1];
    world_bounds.max_corner[1] = bounds.y_limits[1];
    world_bounds.max_corner[2] = bounds.z_limits[1];
    if (!_world) { // first intialization
        _world = world;
        _robot_space = robot_space;
        _control_space = control_space;
        _scene_sdf = std::make_shared<mps::sdf::SceneSDF>(_world);
        b_update_sdf = true;
    } else {
        b_update_sdf |= robot_space->getName() != _robot_space->getName();
        _robot_space = robot_space;
        _control_space = control_space;
        // check whether the sim_env world is different and we need to create a new sdf
        sdf::SceneSDF::SceneSDFConstructionState would_be_state(world, world_bounds);
        b_update_sdf |= would_be_state.isDifferent(_sdf_construction_state);
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

EBDebugDrawerPtr ElasticBandRampComputer::getDebugDrawer() {
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

