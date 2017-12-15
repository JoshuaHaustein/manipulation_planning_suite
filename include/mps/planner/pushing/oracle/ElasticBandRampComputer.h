//
// Created by joshua on 12/11/17.
//

#ifndef MANIPULATION_PLANNING_SUITE_ELASTICBANDRAMPCOMPUTER_H
#define MANIPULATION_PLANNING_SUITE_ELASTICBANDRAMPCOMPUTER_H

#include <mps/planner/pushing/oracle/Oracle.h>
#include <mps/planner/ompl/control/RampVelocityControl.h>
#include <mps/planner/ompl/state/SimEnvState.h>
#include <sim_env/SimEnv.h>
#include <mps/sdf/SDF.h>

namespace mps {
    namespace planner {
            namespace pushing {
                namespace oracle {
                    class EBDebugDrawer;
                    typedef std::shared_ptr<EBDebugDrawer> EBDebugDrawerPtr;

                    class ElasticBandRampComputer : public RobotOracle {
                    public:
                        ElasticBandRampComputer();
                        ~ElasticBandRampComputer() override;

                        void steer(const Eigen::VectorXf &current_robot_state,
                                   const Eigen::VectorXf &desired_robot_state,
                                   std::vector<Eigen::VectorXf> &control_params) const override;

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
                                  ompl::state::SimEnvObjectConfigurationSpacePtr robot_space,
                                  ompl::control::RampVelocityControlSpacePtr control_space,
                                  const mps::planner::ompl::state::PlanningSceneBounds& bounds,
                                  float sdf_resolution,
                                  float error_threshold=0.1f,
                                  bool force_new_sdf=false);
                        EBDebugDrawerPtr getDebugDrawer();
                        void renderSDF(float resolution);

                    private:
                        ompl::control::RampVelocityControlSpacePtr _control_space;
                        ompl::state::SimEnvObjectConfigurationSpacePtr _robot_space;
                        ompl::control::RampVelocityControl* _ramp_control;
                        mps::sdf::SceneSDF::SceneSDFConstructionState _sdf_construction_state;
                        mps::sdf::SceneSDFPtr _scene_sdf;
                        sim_env::WorldPtr _world;
                        EBDebugDrawerPtr _debug_drawer;
                    };

                    typedef std::shared_ptr<ElasticBandRampComputer> ElasticBandRampComputerPtr;
                    typedef std::shared_ptr<const ElasticBandRampComputer> ElasticBandRampComputerConstPtr;
                    typedef std::weak_ptr<ElasticBandRampComputer> ElasticBandRampComputerWeakPtr;
                    typedef std::weak_ptr<const ElasticBandRampComputer>ElasticBandRampComputerWeakConstPtr;

                    class EBDebugDrawer {
                        public:
                            EBDebugDrawer(sim_env::WorldViewerPtr viewer);
                            ~EBDebugDrawer();
                            void renderSDF(mps::sdf::SceneSDFPtr sdf, float resolution);
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
