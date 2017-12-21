//
// Created by joshua on 12/11/17.
//

#ifndef MANIPULATION_PLANNING_SUITE_ELASTICBANDRAMPCOMPUTER_H
#define MANIPULATION_PLANNING_SUITE_ELASTICBANDRAMPCOMPUTER_H

#include <mps/planner/pushing/oracle/Oracle.h>
#include <mps/planner/pushing/oracle/RampComputer.h>
#include <mps/planner/ompl/state/SimEnvState.h>
#include <sim_env/SimEnv.h>
#include <mps/sdf/SDF.h>
#include <list>
#include <functional>

namespace mps {
    namespace planner {
            namespace pushing {
                namespace oracle {
                    class EBDebugDrawer;
                    typedef std::shared_ptr<EBDebugDrawer> EBDebugDrawerPtr;

                    /**
                     * This class provides a robot oracle of the special case where the robot is
                     * a planar holonomic robot. For this type of robot this class implements
                     * an elastic band method that is intialized from a straigt line path and then locally
                     * moved out of collision using a signed distance field. This oracle assumes a
                     * semi-dynamic world, the elastic band is used to compute a path. Thereafter,
                     * each waypoint on this path is connected by computing a ramp control.
                     */
                    class ElasticBandRampComputer : public RobotOracle {
                    public:
                        struct Parameters {
                            float step_size; // determines distance between individual waypoints
                            float min_euclidean_dist; // determines minimal required distance between a pre-predecessor and waypoint need to be
                            float min_radian_dist; // determines minimal required distance for angle between pre-predecessor and waypoint
                            float safety_margin; // determines minimal distance to obstacles
                            // float min_delta_movement; // determines minimal distance between to successive waypoints
                            float gamma; // weight of obstacle potential
                            // float collision_step_size; // determines step size in gradient descent for collision avoidance
                            // float smoothness_step_size; // determines step size in gradient descent for smoothness
                            float iterations_multiplier; // determines maximal ratio between length of collision avoidance path and straight line path
                            float oscillation_threshold; // determines maximum allowed dot product between grad and -prev_grad
                            float look_ahead_weight; // determines mixture between current gradient and look-ahead-gradient
                            Parameters();
                            Parameters(const Parameters& other);
                            ~Parameters();
                            Parameters& operator=(const Parameters& other);
                        };

                        ElasticBandRampComputer(const Parameters& params=Parameters());
                        ~ElasticBandRampComputer() override;

                        void steer(const Eigen::VectorXf &current_robot_state,
                                   const Eigen::VectorXf &desired_robot_state,
                                   std::vector<Eigen::VectorXf> &control_params) const override;

                        void steer(const ompl::state::SimEnvObjectState* current_robot_state,
                                   const ompl::state::SimEnvObjectState* desired_robot_state,
                                   const ompl::state::SimEnvWorldState* current_world_state,
                                   std::vector<Eigen::VectorXf>& control_params) const override;

                        /**
                         * Initializes or reinitializes this elastic band ramp computer.
                         * This function needs to be called before the steering function of this RobotOracle
                         * can be used. If this instance has already been intialized and this function
                         * is called again, the internal data structures are updated with the new parameters.
                         * If the provided world has changed in the number of objects, any name of any object, the pose
                         * of any static object or the robot associated with the provided robot_space, the signed distance
                         * field used by this oracle is recomputed. If none of the above changed, the signed distance field
                         * is reused, saving significant computational resources.
                         */
                        void init(sim_env::WorldPtr world,
                                  sim_env::RobotPtr robot,
                                  ompl::state::SimEnvWorldStateSpacePtr world_state_space,
                                  ompl::control::RampVelocityControlSpacePtr control_space,
                                  const mps::planner::ompl::state::PlanningSceneBounds& bounds,
                                  float sdf_resolution,
                                  float error_threshold=0.1f,
                                  bool force_new_sdf=false);
                        EBDebugDrawerPtr getDebugDrawer() const;
                        void renderSDF(float resolution);

                        Parameters _parameters;

                    private:
                        ompl::state::SimEnvWorldStateSpacePtr _world_state_space;
                        mutable std::vector<sim_env::Ball> _balls;
                        Eigen::VectorXf _gradient_deltas;
                        sim_env::RobotPtr _robot;
                        std::unique_ptr<oracle::RampComputer> _ramp_computer;
                        mps::sdf::SceneSDF::SceneSDFConstructionState _sdf_construction_state;
                        mutable mps::sdf::SceneSDFPtr _scene_sdf;
                        sim_env::WorldPtr _world;
                        mutable EBDebugDrawerPtr _debug_drawer;

                        void computePath(const ompl::state::SimEnvObjectState* start_state,
                                         const ompl::state::SimEnvObjectState* goal_state,
                                         const ompl::state::SimEnvWorldState* world_state,
                                         std::list<Eigen::VectorXf>& waypoints) const;

                        // computes the goal gradient
                        float computeGoalPotential(const Eigen::VectorXf& robot_config,
                                                   const Eigen::VectorXf& goal_config) const;
                        // computes the gradient of the goal potential
                        void computeGoalGradient(const Eigen::VectorXf& robot_config,
                                                 const Eigen::VectorXf& goal,
                                                 Eigen::VectorXf& gradient) const;
                        // computes the obstacle potential at robot_config, for debugging
                        float computeObstaclePotential(const Eigen::VectorXf& robot_config) const;
                        /**
                         *  Computes the gradient of the obstacle potential.
                         *  Returns distance to closest obstacle for performance reasons
                         */
                        float computeObstacleGradient(const Eigen::VectorXf& robot_config,
                                                      Eigen::VectorXf& gradient) const;
                        // computes the potential, just used for debugging
                        float computePotential(const Eigen::VectorXf& config,
                                               const Eigen::VectorXf& goal_config) const;
                        // computes the gradient of the potential at the given configuration, for the given goal
                        // returns the distance to the closest obstacle (for performance reasons)
                        float computeGradient(const Eigen::VectorXf& config,
                                             const Eigen::VectorXf& goal,
                                             Eigen::VectorXf& gradient) const;
                        // computes the distance to the closest obstacle at the given configuration
                        float computeObstacleDistance(const Eigen::VectorXf& config) const;
                        // computes the gradient of the distance to closest obstacle w.r.t to robot configuration
                        void computeObstacleDistanceGradient(const Eigen::VectorXf& config,
                                                              Eigen::VectorXf& gradient) const;
                    };

                    typedef std::shared_ptr<ElasticBandRampComputer> ElasticBandRampComputerPtr;
                    typedef std::shared_ptr<const ElasticBandRampComputer> ElasticBandRampComputerConstPtr;
                    typedef std::weak_ptr<ElasticBandRampComputer> ElasticBandRampComputerWeakPtr;
                    typedef std::weak_ptr<const ElasticBandRampComputer>ElasticBandRampComputerWeakConstPtr;
                    typedef std::function<float(const Eigen::VectorXf&)> PotentialFunction;
                    class EBDebugDrawer {
                        public:
                            EBDebugDrawer(sim_env::WorldViewerPtr viewer);
                            ~EBDebugDrawer();
                            void renderSDF(mps::sdf::SceneSDFPtr sdf, float resolution);
                            // this is special for the 2d case
                            void renderPotential(const sim_env::BoundingBox& range, float resolution,
                                                 float robot_orientation,
                                                 PotentialFunction potential_fn);
                            void drawBalls(std::vector<sim_env::Ball>& balls);
                            void drawPath(std::list<Eigen::VectorXf>& waypoints);
                            void clearSDFRendering();
                            void clear();
                        private:
                            sim_env::WorldViewerPtr _world_viewer;
                            std::vector<sim_env::WorldViewer::Handle> _handles;
                            sim_env::WorldViewer::Handle _sdf_drawing_handle;


                    };

            }
        }
    }
}
#endif //MANIPULATION_PLANNING_SUITE_ELASTICBANDRAMPCOMPUTER_H
