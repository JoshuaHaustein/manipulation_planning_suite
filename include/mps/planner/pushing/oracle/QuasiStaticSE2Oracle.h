#pragma once

#include <mps/planner/pushing/oracle/Oracle.h>
#include <mps/planner/util/Random.h>

namespace mps {
namespace planner {
    namespace pushing {
        namespace oracle {
            /**
             * Hand designed pushing oracle for holonomic 2D robots and objects in SE(2).
             * This oracle is based on Zhou and Mason's work 
             * "Pushing revisited: Differential flatness, trajectory planning and stabilization".
             * It provides a policy for stable pushing of polygonal objects.
             */
            class QuasiStaticSE2Oracle : public PushingOracle {
            public:
                struct Parameters {
                    Parameters(); // creates default values
                    float eps_min; // minimal half width of a pushing edge
                    float eps_dist; // distance between pusher and object from which on to consider both to be in contact
                };

                /**
                 * Create a new quasi static pushing oracle.
                 * @param objects - a list of all objects (including the robot) in the same order as in the
                 *  SimEnvWorldStateSpace used when querying this oracle.
                 * @param robot_id - index of the robot in objects
                 */
                QuasiStaticSE2Oracle(const std::vector<sim_env::ObjectPtr>& objects, unsigned int robot_id);
                ~QuasiStaticSE2Oracle();

                /**
                     * Predicts what action should be taken in the current robot state
                     * in order to move the object from its current state to the desired target state.
                     * @param current_state - current state
                     * @param next_state - desired next state
                     * @param obj_id - object id identifying the object to be pushed
                     * @param control - output control that is directed towards next_state
                     */
                void predictAction(const mps::planner::ompl::state::SimEnvWorldState* current_state,
                    const mps::planner::ompl::state::SimEnvWorldState* target_state,
                    const unsigned int& obj_id, ::ompl::control::Control* control) override;

                /**
                     * Samples a robot state that is likely to be feasible for applying this oracle.
                     * In other words, this function samples from the feasibility distribution.
                     * @param current_obj_state - current object state
                     * @param next_obj_state - desired future object state
                     * @param obj_id - object id identifying the object to be pushed
                     * @param new_robot_state  - current robot state
                     */
                void samplePushingState(const mps::planner::ompl::state::SimEnvWorldState* current_state,
                    const mps::planner::ompl::state::SimEnvWorldState* next_state,
                    const unsigned int& obj_id,
                    mps::planner::ompl::state::SimEnvObjectState* new_robot_state) override;

            protected:
                struct RobotPushingEdge {
                    Eigen::Vector2f normal; // in robot frame
                    Eigen::Vector2f from; // in robot frame
                    Eigen::Vector2f to; // in robot frame
                    Eigen::Vector2f center; // from + to / 2.0
                    Eigen::Vector2f dir; // dir = to - from
                    float edge_length; // norm of dir
                };

                typedef std::shared_ptr<RobotPushingEdge> RobotPushingEdgePtr;

                struct ObjectPushingEdge {
                    Eigen::Vector2f normal; // in object frame
                    Eigen::Vector2f from; // in object frame
                    Eigen::Vector2f to; // in object frame
                    Eigen::Vector2f dir; // dir = to - from / ||to - from||
                    Eigen::Vector2f pcom; // projection of center of mass on the line x = from + t * dir with t in [0, 1]
                    float edge_length;
                    // angle at which a pusher should be placed in object frame (atan2(-normal[1], -normal[0]))
                    float pushing_angle;
                };

                typedef std::shared_ptr<ObjectPushingEdge> ObjectPushingEdgePtr;

                struct PushingEdgePair {
                    RobotPushingEdgePtr robot_edge;
                    ObjectPushingEdgePtr object_edge;
                    float max_translation; // max translational offset (>0) from object_edge->pcom along object_edge->dir
                    float min_translation; // min translational offset (<0) from object_edge->pcom along object_edge->dir
                };
            private:
                unsigned int _robot_id;
                std::vector<sim_env::ObjectPtr> _objects;
                ::ompl::RNGPtr _random_gen;
                std::vector<RobotPushingEdgePtr> _robot_edges;
                std::vector<std::vector<ObjectPushingEdgePtr>> _obj_edges;
                std::vector<std::vector<PushingEdgePair>> _contact_pairs;
                Parameters _params;

                mutable Eigen::VectorXf _eigen_config;
                mutable Eigen::VectorXf _eigen_config2;

                void computeRobotPushingEdges();
                void computeObjectPushingEdges();
                float computeSamplingWeights(const ompl::state::SimEnvWorldState* current_state, 
                                        const ompl::state::SimEnvWorldState* next_state, unsigned int obj_id,
                                        std::vector<float>& sampling_weights) const;
            };
        }
    }
}
}