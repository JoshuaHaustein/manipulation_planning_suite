#include <mps/sdf/SDF.h>
#include <mps/planner/util/Math.h>
#include <mps/planner/util/Logging.h>
#include <fstream>
#include <yaml-cpp/yaml.h>
#include <sim_env/utils/YamlUtils.h>
#include <boost/filesystem/path.hpp>
#include <stack>

using namespace mps::sdf;
using namespace mps::planner::util::math;
using namespace sim_env;

SDF::SDF() : _grid(Eigen::Vector3f(0.0f, 0.0f, 0.0f), Eigen::Vector3f(0.0f, 0.0f, 0.0f), 1.0f) {
}

SDF::~SDF() = default;
SDF::SDF(const SDF& other) = default;
SDF& SDF::operator=(const SDF& other) = default;

void SDF::load(const std::string& filename) {
    std::fstream fs;
    fs.open(filename, std::fstream::in | std::fstream::binary);
    fs >> _grid;
    fs >> _approximation_box.min_corner[0] >> _approximation_box.min_corner[1] >> _approximation_box.min_corner[2];
    fs >> _approximation_box.max_corner[0] >> _approximation_box.max_corner[1] >> _approximation_box.max_corner[2];
}

void SDF::save(const std::string& filename) const {
    std::fstream fs;
    fs.open(filename, std::fstream::out | std::fstream::binary);
    fs << _grid;
    fs << _approximation_box.min_corner[0] << _approximation_box.min_corner[1] << _approximation_box.min_corner[2];
    fs << _approximation_box.max_corner[0] << _approximation_box.max_corner[1] << _approximation_box.max_corner[2];
    fs.close();
}

void SDF::computeSDF(sim_env::WorldPtr world, const sim_env::BoundingBox& aabb,
                     float cell_size, bool ignore_robots, const std::vector<std::string>& ignore_list)
{
    _grid = grid::VoxelGrid<float, float>(aabb.min_corner, aabb.max_corner, cell_size, 0.0f);
    grid::VoxelGrid<float, int> collision_map(aabb.min_corner, aabb.max_corner, cell_size, 1);
    computeSCM(collision_map, world, ignore_robots, ignore_list);
    mps::sdf::fmm::computeSDF<float>(collision_map, _grid, cell_size);
    _approximation_box = aabb;
}

void SDF::setTransform(const Eigen::Affine3f& tf) {
    _grid.setTransform(tf);
}

Eigen::Affine3f SDF::getTransform() const {
    return _grid.getTransform();
}

float SDF::getHeuristicDistance(const Eigen::Vector3f& pos) const {
    grid::UnsignedIndex idx;
    Eigen::Vector3f local_pos;
    _grid.mapToGrid(pos, local_pos, idx);
    return getLocalHeuristicDistance(pos);
}

float SDF::getDistance(const Eigen::Vector3f& pos) const {
    Eigen::Vector3f local_pos;
    grid::UnsignedIndex idx;
    bool valid_idx = _grid.mapToGrid(pos, local_pos, idx);
    if (valid_idx) {
        return _grid(idx);
    }
    return getLocalHeuristicDistance(local_pos);
}

void SDF::setApproximationBox(const sim_env::BoundingBox& aabb) {
    _approximation_box = aabb;
}

float SDF::getLocalHeuristicDistance(const Eigen::Vector3f& pos) const {
    Eigen::Vector3f projected_pos(clamp(pos[0], _approximation_box.min_corner[0], _approximation_box.max_corner[0]),
                                  clamp(pos[1], _approximation_box.min_corner[1], _approximation_box.max_corner[1]),
                                  clamp(pos[2], _approximation_box.min_corner[2], _approximation_box.max_corner[2]));
    return (projected_pos - pos).norm();
}

void SDF::computeSCM(grid::VoxelGrid<float, int>& collision_map, sim_env::WorldPtr world,
                     bool ignore_robots, const std::vector<std::string>& ignore_list)
{
    // first create a set so we can quickly look up whether we have to ignore an objects
    std::unordered_set<std::string> ignore_set;
    for (auto ignore_name : ignore_list) {
        ignore_set.insert(ignore_name);
    }
    // next create init our data structures for our algorithm
    grid::UnsignedIndex min_idx(0, 0, 0);
    grid::UnsignedIndex max_idx(_grid.getXSize(), _grid.getYSize(), _grid.getZSize());
    std::stack<std::pair<grid::UnsignedIndex, grid::UnsignedIndex> > index_stack;
    index_stack.emplace(std::make_pair(min_idx, max_idx));
    // loop over index ranges that are in collision until none are left
    while (not index_stack.empty()) {
        auto index_pair = index_stack.top();
        index_stack.pop();
        auto& min_idx = index_pair.first;
        auto& max_idx = index_pair.second;
        auto box_size = max_idx - min_idx;
        if (box_size.ix == 1 and box_size.iy == 1 and box_size.iz == 1) {
            // Base case, we are looking at only one cell, store collision in this cell
            collision_map(min_idx) = -1.0;
        } else {
            // else we need to split this cell up and see which child ranges are in collision
            std::array<std::array<size_t, 2>, 3> half_sizes;
            splitBox(box_size, half_sizes);
            std::array<std::array<size_t, 2>, 3> idx_offsets = {{{{0, half_sizes[0][0]}},
                                                                {{0, half_sizes[1][0]}},
                                                                {{0, half_sizes[2][0]}}}};
            // now we create the actual ranges for each of the 8 children
            for (unsigned int ix = 0; ix < 2; ++ix) {
                for (unsigned int iy = 0; iy < 2; ++iy) {
                    for (unsigned int iz = 0; iz < 2; ++iz) {
                        // if the current child range actually has some volume/cells
                        if (half_sizes[0][ix] * half_sizes[1][iy] * half_sizes[2][iz]) {
                            // create indices that describe the current child
                            grid::UnsignedIndex child_min_idx(idx_offsets[0][ix], idx_offsets[1][iy],
                                                              idx_offsets[2][iz]);
                            grid::UnsignedIndex child_max_idx(idx_offsets[0][ix] + half_sizes[0][ix],
                                                            idx_offsets[1][iy] + half_sizes[1][iy],
                                                            idx_offsets[2][iz] + half_sizes[2][iz]);
                            child_min_idx += min_idx;
                            child_max_idx += min_idx;
                            // create the bounding box that covers this child
                            sim_env::BoundingBox query_aabb;
                            collision_map.getCellPosition(child_min_idx, query_aabb.min_corner, false);
                            collision_map.getCellPosition(child_max_idx, query_aabb.max_corner, false);
                            // query the world for colliding objects
                            std::vector<sim_env::ObjectPtr> colliding_objects;
                            world->getObjects(query_aabb, colliding_objects, ignore_robots);
                            // check whether any of those can not be ignored
                            bool b_collision = false;
                            for (auto obj : colliding_objects) {
                                // if this object is not in our ignore list, we have a collision
                                b_collision |= ignore_set.count(obj->getName()) == 0;
                            }
                            if (b_collision) {
                                index_stack.emplace(std::make_pair(child_min_idx, child_max_idx));
                            }
                        }
                    }
                }
            }
        }
    }
}

void SDF::splitBox(const sim_env::grid::UnsignedIndex& box_size,
                std::array<std::array<size_t, 2>, 3>& half_sizes) const
{
    half_sizes[0][0] = box_size.ix / 2; // we split this box into 8 children by dividing each axis by 2
    half_sizes[0][1] = box_size.ix - half_sizes[0][0];
    half_sizes[1][0] = box_size.iy / 2;
    half_sizes[1][1] = box_size.iy - half_sizes[1][0];
    half_sizes[2][0] = box_size.iz / 2;
    half_sizes[2][1] = box_size.iz - half_sizes[2][0];
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////// SceneSDFConstructionState //////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
SceneSDF::SceneSDFConstructionState::SceneSDFConstructionState() = default;

SceneSDF::SceneSDFConstructionState::SceneSDFConstructionState(const SceneSDFConstructionState& other) = default;

SceneSDF::SceneSDFConstructionState::SceneSDFConstructionState(sim_env::WorldPtr world, const sim_env::BoundingBox& aabb) {
    set(world, aabb);
}

SceneSDF::SceneSDFConstructionState& SceneSDF::SceneSDFConstructionState::operator=(const SceneSDFConstructionState& other) = default;

void SceneSDF::SceneSDFConstructionState::set(sim_env::WorldPtr world, const sim_env::BoundingBox& aabb) {
    std::vector<sim_env::ObjectPtr> objects;
    world->getObjects(objects, false);
    for (auto object : objects) {
        if (object->isStatic()) {
            _static_tfs[object->getName()] = object->getTransform();
        }
        if (object->getType() == sim_env::EntityType::Robot) {
            _robot_names.insert(object->getName());
        } else {
            _object_names.insert(object->getName());
        }
    }
    _aabb = aabb;
}

bool SceneSDF::SceneSDFConstructionState::isDifferent(const SceneSDFConstructionState& other,
                                                      const std::vector<std::string>& ignore_list)
{
    std::unordered_set<std::string> ignore_set;
    if (not ignore_list.empty()) {
        ignore_set.insert(ignore_list.begin(), ignore_list.end());
    }
    // first check whether we have the same objects
    for (auto object_name : _object_names) {
        if (!other._object_names.count(object_name)) {
            if (!ignore_set.count(object_name)) {
                return true;
            }
        }
    }
    // next check whether we have the same robots
    for (auto robot_name : _robot_names) {
        if (!other._robot_names.count(robot_name)) {
            if (!ignore_set.count(robot_name)) {
                return true;
            }
        }
    }
    // finally check whether all statics have the same tfs
    for (auto key_value_pair : _static_tfs) {
        auto other_iter = other._static_tfs.find(key_value_pair.first);
        // if this static doesn't exist in the other, we are different
        if (other_iter == other._static_tfs.end()) {
            return true; // this should actually never happen due to the tests above
        }
        // else
        if (not key_value_pair.second.isApprox(other_iter->second)) {
            return true;
        }
    }
    return false;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////// SceneSDF ///////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
SceneSDF::SceneSDF(sim_env::WorldPtr world) : _world(world), _has_statics(false) {
}

SceneSDF::~SceneSDF() = default;

bool SceneSDF::load(const std::string& filename) {
    static const std::string log_prefix("mps::sdf::SceneSDF::load");
    // first reset sdf
    reset();
    // setup
    bool success = true;
    boost::filesystem::path boost_path(filename);
    auto dir_path = boost_path.parent_path();
    YAML::Node base_node = YAML::Load(filename);
    // let's see what objects the world has
    std::vector<sim_env::ObjectPtr> objects;
    _world->getObjects(objects, false);
    for (auto object : objects) {
        if (object->isStatic()) {
            _has_statics = true;
            continue;
        }
        if (base_node[object->getName()]) {
            _movables.push_back(object);
            _movable_sdfs.push_back(SDF());
             boost::filesystem::path movable_filename(dir_path);
             movable_filename += base_node[object->getName()].as<std::string>();
            _movable_sdfs[_movable_sdfs.size() - 1].load(movable_filename.string());
        } else {
            mps::planner::util::logging::logErr("Could not load SDF for object " + object->getName(), log_prefix);
            success = false;
        }
    }
    if (_has_statics) {
        if (base_node["___static_sdf___"]) {
            boost::filesystem::path static_filename(dir_path);
            static_filename += base_node["___static_sdf___"].as<std::string>();
            _static_sdf.load(static_filename.string());
        } else {
            mps::planner::util::logging::logErr("Could not load SDF for statics", log_prefix);
            success = false;
        }
    }
    _bounding_box.min_corner = base_node["___min_point___"].as<Eigen::Vector3f>();
    _bounding_box.max_corner = base_node["___max_point___"].as<Eigen::Vector3f>();
    return success;
}

void SceneSDF::save(const std::string& filename) const {
    boost::filesystem::path yaml_file_path(filename);
    auto dir_path = yaml_file_path.parent_path();
    auto sdf_name = yaml_file_path.filename();
    YAML::Emitter out;
    out << YAML::BeginMap;
    if (_has_statics) {
        std::string static_filename(sdf_name.string() + ".static.sdf");
        boost::filesystem::path static_path(dir_path);
        static_path += static_filename;
        out << YAML::Key << "___static_sdf___";
        out << YAML::Value << static_filename;
        _static_sdf.save(static_path.string());
    }
    for (size_t i = 0; i < _movables.size(); ++i) {
        auto movable = _movables[i].lock();
        if (!movable) throw std::logic_error("Could not access movable. Weak pointer expired!");
        std::string movable_name = movable->getName();
        std::string movable_filename(sdf_name.string() + "." + movable_name + ".sdf");
        boost::filesystem::path movable_path(dir_path);
        dir_path += movable_filename;
        out << YAML::Key << movable_name;
        out << YAML::Value << movable_filename;
        _movable_sdfs[i].save(movable_path.string());
    }
    out << YAML::Key << "___min__point___";
    YAML::convert<Eigen::Vector3f> converter;
    out << YAML::Value << converter.encode(_bounding_box.min_corner);
    out << YAML::Key << "___max__point___";
    out << YAML::Value << converter.encode(_bounding_box.max_corner);
    out << YAML::EndMap;
    std::fstream fs(yaml_file_path.string(), std::fstream::out);
    fs << out.c_str();
    fs.close();
}

SceneSDF::SceneSDFConstructionState SceneSDF::computeSDF(const sim_env::BoundingBox& aabb, float static_cell_size,
                                                         float movable_cell_size, float max_approx_error,
                                                         const StrToFloatMap& radii,
                                                         const StrToStrMap& movable_sdfs)
{
    static const std::string log_prefix("[mps::sdf::SceneSDF::computeSDF]");
    auto logger = _world->getLogger();
    logger->logDebug("Resetting", log_prefix);
    reset();
    SceneSDFConstructionState construction_state(_world, aabb);
    // first save world state
    _world->saveState();
    _bounding_box = aabb;
    // classify objects based on whether they are static or not
    logger->logDebug("Classifying objects", log_prefix);
    std::vector<std::string> static_obstacles;
    std::vector<std::string> movable_objects;
    std::vector<sim_env::ObjectPtr> objects;
    _world->getObjects(aabb, objects, true);
    for (auto object : objects) {
        if (object->isStatic()) {
            static_obstacles.push_back(object->getName());
        } else {
            movable_objects.push_back(object->getName());
            _movables.push_back(object);
            _movable_sdfs.push_back(SDF());
        }
    }
    // now, let's create the static sdf first
    _has_statics = not static_obstacles.empty();
    if (_has_statics) {
        logger->logDebug("Creating static SDF", log_prefix);
        _static_sdf.computeSDF(_world, aabb, static_cell_size, true, movable_objects);
    }

    // next create an sdf for each movable object
    Eigen::Affine3f identity_tf;
    identity_tf.setIdentity();
    if (not movable_objects.empty()) { // if we have movable objects
        logger->logDebug("Creating movable SDFs", log_prefix);
        // create ignore list that always contains all object names except for the movable we create an sdf for
        std::vector<std::string> ignore_list;
        if (not static_obstacles.empty()) {
            ignore_list.insert(ignore_list.begin(), static_obstacles.begin(), static_obstacles.end());
        }
        size_t offset = ignore_list.size();
        for (auto movable_name : movable_objects) {
            ignore_list.push_back(movable_name);
        }
        for (size_t m = 0; m < movable_objects.size(); ++m) { // run over all movable objects
            auto current_obj_name = movable_objects[m];
            if (movable_sdfs.count(current_obj_name)) { // if we have an sdf for it, load it
                logger->logDebug("Loading sdf for object " + current_obj_name, log_prefix);
                _movable_sdfs[m].load(movable_sdfs.at(current_obj_name));
            } else { // else create a new one
                // move the current object name to the end of the ignore list
                assert(current_obj_name == ignore_list[m + offset]);
                std::swap(ignore_list[m + offset], ignore_list[ignore_list.size() - 1]);
                ignore_list.pop_back(); // temporary delete current_obj_name from ignore list
                // next align object and world frame
                auto movable = _movables[m].lock();
                if (!movable) throw std::logic_error("Could not acquire access to object. Weak pointer expired");
                auto orig_tf = movable->getTransform();
                movable->setTransform(identity_tf);
                // compute the size we need for the sdf
                auto object_aabb = movable->getLocalAABB();  // TODO this is configuration dependent -> we actually need SDFs for each link
                float radius = 0.0f;
                if (radii.count(current_obj_name)) {
                    radius = radii.at(current_obj_name);
                }
                auto sdf_aabb = computeSDFSize(object_aabb, max_approx_error, radius);
                // now we are ready to compute the sdf for this object
                logger->logDebug("Creating SDF for object " + current_obj_name, log_prefix);
                _movable_sdfs[m].computeSDF(_world, sdf_aabb, movable_cell_size, true, ignore_list);
                _movable_sdfs[m].setApproximationBox(object_aabb);
                // after this, clean up by resetting the tf and fixing ignore list again
                ignore_list.push_back(current_obj_name);
                std::swap(ignore_list[m + offset], ignore_list[ignore_list.size() - 1]);
                movable->setTransform(orig_tf);
                assert(current_obj_name == ignore_list[m + offset]);
            }
        }
    }
    _world->restoreState();
    logger->logDebug("SDF creation finished.", log_prefix);
    return construction_state;
}

float SceneSDF::getDistance(const Eigen::Vector3f& position)
{
    float min_distance = std::numeric_limits<float>::max();
    for (size_t i = 0; i < _movables.size(); ++i) {
        auto movable = _movables[i].lock();
        auto tf = movable->getTransform();
        _movable_sdfs[i].setTransform(tf);
        min_distance = std::min(min_distance, _movable_sdfs[i].getDistance(position));
    }
    if (_has_statics) {
        min_distance = std::min(min_distance, _static_sdf.getDistance(position));
    }
    return min_distance;
}

void SceneSDF::getDistances(const std::vector<Eigen::Vector3f>& positions, std::vector<float>& distances)
{
    distances.resize(positions.size(), std::numeric_limits<float>::max());
    for (size_t i = 0; i < _movables.size(); ++i) {
        auto movable = _movables[i].lock();
        auto tf = movable->getTransform();
        _movable_sdfs[i].setTransform(tf);
        for (size_t j = 0; j < positions.size(); ++j) {
            distances[j] = std::min(distances[j], _movable_sdfs[i].getDistance(positions[j]));
        }
    }
    if (_has_statics) {
        for (size_t j = 0; j < positions.size(); ++j) {
            distances[j] = std::min(distances[j], _static_sdf.getDistance(positions[j]));
        }
    }
}

void SceneSDF::updateTransforms() {
    for (size_t i = 0; i < _movables.size(); ++i) {
        auto movable = _movables[i].lock();
        auto tf = movable->getTransform();
        _movable_sdfs[i].setTransform(tf);
    }
}

float SceneSDF::getDistanceDirty(const Eigen::Vector3f& position) {
    float min_distance = std::numeric_limits<float>::max();
    for (auto& movable_sdf : _movable_sdfs) {
        min_distance = std::min(min_distance, movable_sdf.getDistance(position));
    }
    if (_has_statics) {
        min_distance = std::min(min_distance, _static_sdf.getDistance(position));
    }
    return min_distance;
}

sim_env::BoundingBox SceneSDF::getBoundingBox() const {
    return _bounding_box;
}

sim_env::BoundingBox SceneSDF::computeSDFSize(const sim_env::BoundingBox& aabb, float approx_error, float radius) const
{
    auto extents = aabb.extents();
    auto center = aabb.center();
    float scaling_factor = (1.0f - (1.0f - approx_error) * radius / extents.norm()) / approx_error;
    auto scaled_extents = scaling_factor * extents;
    sim_env::BoundingBox new_aabb;
    new_aabb.min_corner = center - scaled_extents;
    new_aabb.max_corner = center + scaled_extents;
    return new_aabb;
}

void SceneSDF::reset() {
    _movables.clear();
    _movable_sdfs.clear();
    _has_statics = false;
}