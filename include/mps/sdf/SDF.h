#ifndef MANIPULATION_PLANNING_SUITE_SDF_H
#define MANIPULATION_PLANNING_SUITE_SDF_H

#include <sim_env/SimEnv.h>
#include <mps/sdf/Grid.h>
#include <mps/sdf/FMM.h>
#include <unordered_map>
#include <Eigen/Geometry>

namespace mps {
    namespace sdf {
        /**
         * This class represents a signed distance field.
         * There are two ways to fill this class with meaningful values:
         *  1. load a previously computed sdf
         *  2. compute the values from a sim_env world.
         */
        class SDF {
            public:
                SDF();
                ~SDF();
                SDF(const SDF& other);
                SDF& operator=(const SDF& other);

                /**
                 * Loads the distances for this sdf from file.
                 */
                void load(const std::string& filename);
                /**
                 * Saves this sdf to file, so it can be loaded with load again.
                 */
                void save(const std::string& filename) const;

                /**
                 * Computes the values for this sdf from the current state of the given sim_env world.
                 * The sdf is created for a specified volume. You may later change the transform of this sdf
                 * which allows you to create object specific SDFs, i.e. you can create an sdf from
                 * an aabb that encopes an object which is placed at the origin. As a result
                 * you then get an sdf that is defined in the object frame and can be moved later on.
                 * NOTE: If there are no collisions with any obstacle in the specified volume, an exception
                 * is thrown. TODO: This behavior might be undesirable, instead we could set this SDF to
                 * contain infinite distances.
                 *
                 * @param world - sim_env world to compute sdf for
                 * @param aabb - axis aligned bounding volume to create sdf for
                 * @param cell_size - size of voxel cells
                 * @param ignore_robots - if true, robots are ignored in the computation
                 * @param ignore_list - objects to ignore in the sdf computation.
                 */
                void computeSDF(sim_env::WorldPtr world, const sim_env::BoundingBox& aabb, float cell_size,
                                bool ignore_robots=true,
                                const std::vector<std::string>& ignore_list=std::vector<std::string>());

                void setTransform(const Eigen::Affine3f& tf);
                Eigen::Affine3f getTransform() const;

                /**
                 * Returns a heuristical shortest distance of the given point to the closest obstacle surface.
                 * This heuristic is intended to be used for points outside of the grid and is
                 * the distance to an approximating bounding box of all obstacles. This is particularly
                 * meaningful if this SDF represents only a single object.
                 * @param point - point as Eigen vector (x, y, z)
                 */
                float getHeuristicDistance(const Eigen::Vector3f& pos) const;

                /**
                 * Returns the shortest distance of the given point (in world frame)
                 * to the closest obstacle surface.
                 */
                float getDistance(const Eigen::Vector3f& pos) const;

                /**
                 * Sets a bounding box used to compute approximate distances for points outside
                 * of the underlying grid. Only use when you know what you are doing!
                 */
                void setApproximationBox(const sim_env::BoundingBox& aabb);
            private:
                grid::VoxelGrid<float, float> _grid;
                // used for heuristic distance for query points outside of grid
                sim_env::BoundingBox _approximation_box;
                /*
                 * Returns a heuristical shortest distance of the given point to the closest obstacle surface.
                 * @param pos - point as Eigen vecotr (x, y, z), assumed to be in local frame
                */
                float getLocalHeuristicDistance(const Eigen::Vector3f& pos) const;

                void computeSCM(grid::VoxelGrid<float, int>& collision_map,
                                sim_env::WorldPtr world,
                                bool ignore_robots,
                                const std::vector<std::string>& ignore_list);

                /*
                * Computes a signed collision map recursively.
                * INVARIANT: This function is only called if there is a collision for a box ranging from min_idx to max_idx
                * @param min_idx - numpy array [min_x, min_y, min_z] cell indices
                * @param max_idx - numpy array [max_x, max_y, max_z] cell indices (the box excludes these)
                * @param collision_map - the grid to operate on
                */
                void computeSCMRec(const grid::UnsignedIndex& min_idx, const grid::UnsignedIndex& max_idx,
                                   grid::VoxelGrid<float, int>& collision_map,
                                   sim_env::WorldPtr world,
                                   bool ignore_robots,
                                   const std::unordered_map<std::string, bool>& ignore_map);
        };

        typedef std::shared_ptr<SDF> SDFPtr;
        typedef std::shared_ptr<const SDF> SDFConstPtr;
        typedef std::weak_ptr<SDF> SDFWeakPtr;
        typedef std::weak_ptr<const SDF> SDFConstWeakPtr;

        /*
        * A scene sdf is a signed distance field for a motion planning scene that contains
        * a robot, multiple movable kinbodies and a set of static obstacles.
        * A scene sdf creates separate sdfs for the static obstacles and the movable objects.
        * When querying a scene sdf, the returned distance takes the current state of the environment,
        * i.e. the current poses of all movable kinbodies into account.
        * The robot is not considered in the distance field.
        * NOTE: Multiple robots in a world are not supported!
        */
        class SceneSDF {
            public:
                typedef std::unordered_map<std::string, std::string> StrToStrMap;
                typedef std::unordered_map<std::string, float> StrToFloatMap;
                /**
                 *  Creates a new scene sdf for the given world.
                 */
                SceneSDF(sim_env::WorldPtr world);
                ~SceneSDF();

                /**
                 * Loads a scene sdf from file.
                 * @param filename - filename of a scene sdf file.
                 * @return true - if the scene sdf was loaded successfully, i.e. it has an sdf for static and all movable
                 *                objects (excluding robots)
                 */
                bool load(const std::string& filename);

                /**
                 * Saves tis scene sdf to the given file.
                 * NOTE: This will create multiple files in the same directory, as each movable sdf is stored
                 *      in a separate file.
                 */
                void save(const std::string& filename) const;

                /**
                 * Computes a scene sdf within the given volume.
                 * @param aabb - axis aligned bounding box of the volume this scene sdf is supposed to encompass
                 * @param static_cell_size - cell size of the sdf used to represent static obstacles
                 * @param movable_cell_size - cell size of the sdfs used to represent movable objects
                 * @param max_approx_error - maximum relative error in distance estimation tolerated at edges of movable sdfs
                 * @param radii - a map mapping movable object name to the radius of a circle/ball that is
                 *               fully inscribed in this object. If provided for a movable, the SDF for this movable
                 *               can cover a smaller volume without increasing the relative error for distance estimation
                 *               when transitioning from outside the sdf to inside
                 * @param movable_sdfs - a map providing sdf filenames for individual movable objects
                 */
                void computeSDF(const sim_env::BoundingBox& aabb, float static_cell_size=0.02f,
                                float movable_cell_size=0.02f, float max_approx_error=0.1f,
                                const StrToFloatMap& radii=StrToFloatMap(),
                                const StrToStrMap& movable_sdfs=StrToStrMap());

                /**
                 * Returns the (approximate) signed distance from the given position to the closest
                 * obstacle surface.
                 */
                float getDistance(const Eigen::Vector3f& position);

                /**
                 * Computes the (approximate) signed distances from the given positions to the closest
                 * obstacle surfaces.
                 */
                void getDistances(const std::vector<Eigen::Vector3f>& positions, std::vector<float>& distances);
            private:
                sim_env::WorldPtr _world;
                std::vector<sim_env::ObjectWeakPtr> _movables;
                std::vector<SDF> _movable_sdfs;
                SDF _static_sdf;
                bool _has_statics;

                /*
                 * Computes the required size of an sdf for a movable object such
                 * that at the boundary of the sdf the relative error in distance estimation to the objects's
                 * surface is bounded by approx_error.
                */
                sim_env::BoundingBox computeSDFSize(const sim_env::BoundingBox& aabb,
                                                    float approx_error, float radius=0.0f) const;
                void reset();

        };

        typedef std::shared_ptr<SceneSDF> SceneSDFPtr;
        typedef std::shared_ptr<const SceneSDF> SceneSDFConstPtr;
        typedef std::weak_ptr<SceneSDF> SceneSDFWeakPtr;
        typedef std::weak_ptr<const SceneSDF> SceneSDFConstWeakPtr;
    }
}

#endif //MANIPULATION_PLANNING_SUITE_SDF_H