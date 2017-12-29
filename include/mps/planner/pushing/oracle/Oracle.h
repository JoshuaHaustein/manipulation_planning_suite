#ifndef MPS_PLANNER_ORACLE_ORACLE_H
#define MPS_PLANNER_ORACLE_ORACLE_H

#include <Eigen/Core>
#include <memory>
#include <vector>
#include <mps/planner/util/Time.h>
#include <mps/planner/ompl/state/SimEnvState.h>

namespace mps {
    namespace planner {
        namespace pushing {
            namespace oracle {
                class PushingOracle {
                public:
                    struct ObjectData {
                        float mass;
                        float inertia;
                        float width;
                        float height;
                        float mu;
                    };
                    virtual ~PushingOracle() = 0;
                    void setObjectData(const std::vector<ObjectData>& object_data);
                    // TODO can we make all this functions const?
                    // TODO it would make more sense if these function take SimEnvObjectState* as input rather than Eigen::VectorXf
                    // TODO similarly, it would make sense to pass VelocityControl* as controls
                    /**
                     * Predicts how well the oracle is capable to provide
                     * an action to manipulate an object in current_obj_state such that it moves
                     * towards next_obj_state.
                     * @param current_obj_state - the current state of the object
                     * @param next_obj_state - the desired future state of the object
                     * @param obj_id - integer identifying the object to push
                     * @return propbability
                     */
                    virtual float predictPushability(const Eigen::VectorXf& current_obj_state,
                                                     const Eigen::VectorXf& next_obj_state,
                                                     const unsigned int& obj_id) = 0;

                    /**
                     * Projects the given next_obj_state on to a state output such that the direction
                     * from current_obj_state to output is the same as from current_obj_state to next_obj_state
                     * and the output state is within num_std * standard deviations of pushability away from current_obj_state.
                     * @param current_obj_state - current object state
                     * @param next_obj_state - next object state
                     * @param num_std - multiplier denoting how many stds away from current_obj_state output state can be
                     * @param obj_id - object id identifying the object to be pushed
                     * @param output - projected state
                     */
                    virtual void projectToPushability(const Eigen::VectorXf& current_obj_state,
                                                      const Eigen::VectorXf& next_obj_state,
                                                      const float& num_std,
                                                      const unsigned int& obj_id,
                                                      Eigen::VectorXf& output) = 0;

                    /**
                     * Predicts the feasibility of the oracle given the current robot state, object state
                     * and the desired future object state. Another interpretation of the feasibility
                     * is the probability that the oracle can provide an action achieving the desired object
                     * state space given the current robot state.
                     * @param current_robot_state
                     * @param current_obj_state
                     * @param next_obj_state
                     * @param obj_id - object id identifying the object to be pushed
                     * @return feasibility
                     */
                    virtual float predictFeasibility(const Eigen::VectorXf& current_robot_state,
                                                     const Eigen::VectorXf& current_obj_state,
                                                     const Eigen::VectorXf& next_obj_state,
                                                     const unsigned int& obj_id) = 0;

                    /**
                     * Predicts what action should be taken in the current robot state
                     * in order to move the object from its current state to the desired future state.
                     * @param current_state - current state
                     * @param next_state - desired next state
                     * @param control - output control that is directed towards next_state
                     * @param obj_id - object id identifying the object to be pushed
                     */
                    virtual void predictAction(const Eigen::VectorXf& current_robot_state,
                                               const Eigen::VectorXf& current_obj_state,
                                               const Eigen::VectorXf& next_obj_state,
                                               const unsigned int& obj_id,
                                               Eigen::VectorXf& control) = 0;

                    /**
                     * Samples a robot state that is likely to be feasible for applying this oracle.
                     * In other words, this function samples from the feasibility distribution.
                     * @param current_obj_state - current object state
                     * @param next_obj_state - desired future object state
                     * @param obj_id - object id identifying the object to be pushed
                     * @param new_robot_state  - current robot state
                     */
                    virtual void sampleFeasibleState(const Eigen::VectorXf& current_obj_state,
                                                     const Eigen::VectorXf& next_obj_state,
                                                     const unsigned int& obj_id,
                                                     Eigen::VectorXf& new_robot_state) = 0;

                    virtual float getMaximalPushingDistance() const = 0;

                   /**
                    * Optional timer for adding external cpu time to clock
                    */
                   std::shared_ptr<mps::planner::util::time::Timer> timer;
               protected:
                   std::vector<ObjectData> _object_data;
                };

                typedef std::shared_ptr<PushingOracle> PushingOraclePtr;
                typedef std::shared_ptr<const PushingOracle> PushingOracleConstPtr;
                typedef std::weak_ptr<PushingOracle> PushingOracleWeakPtr;
                typedef std::weak_ptr<const PushingOracle> PushingOracleWeakConstPtr;

                class RobotOracle {
                public:
                    virtual ~RobotOracle() = 0;
                    // TODO same as for the pushing oracle, it would make sense to use SimEnvObjectState* here
                    virtual void steer(const Eigen::VectorXf& current_robot_state,
                                       const Eigen::VectorXf& desired_robot_state,
                                       std::vector<Eigen::VectorXf>& control_params) const = 0;
                    /**
                     * Computes a sequence of controls that attempt to move the robot from
                     * the current state to the desired state given the current state of the world.
                     * The control sequences is parameterized and returned in the vector control_params.
                     */
                    virtual void steer(const ompl::state::SimEnvObjectState* current_robot_state,
                                       const ompl::state::SimEnvObjectState* desired_robot_state,
                                       const ompl::state::SimEnvWorldState* current_world_state,
                                       std::vector<Eigen::VectorXf>& control_params) const = 0;
                };

                typedef std::shared_ptr<RobotOracle> RobotOraclePtr;
                typedef std::shared_ptr<const RobotOracle> RobotOracleConstPtr;
                typedef std::weak_ptr<RobotOracle> RobotOracleWeakPtr;
                typedef std::weak_ptr<const RobotOracle> RobotOracleWeakConstPtr;

            }
        }
    }
}

#endif // MPS_PLANNER_ORACLE_ORACLE_H
