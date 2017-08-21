//
// Created by joshua on 8/14/17.
//

#ifndef MANIPULATION_PLANNING_SUITE_SIMENVSTATE_H
#define MANIPULATION_PLANNING_SUITE_SIMENVSTATE_H

// STL includes
#include <memory>
#include <string>
#include <unordered_map>
// Eigen includes
#include <Eigen/Core>
// ompl includes
#include <ompl/base/StateSpace.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/SpaceInformation.h>
// sim_env includes
#include <sim_env/SimEnv.h>

namespace mps {
    namespace planner {
        namespace ompl {
            namespace state {
                class SimEnvObjectStateSpace;
                typedef std::shared_ptr<SimEnvObjectStateSpace> SimEnvObjectStateSpacePtr;
                typedef std::weak_ptr<SimEnvObjectStateSpace> SimEnvObjectStateSpaceWeakPtr;
                typedef std::shared_ptr<const SimEnvObjectStateSpace> SimEnvObjectStateSpaceConstPtr;
                typedef std::weak_ptr<const SimEnvObjectStateSpace> SimEnvObjectStateSpaceConstWeakPtr;

                class SimEnvWorldStateSpace;
                typedef std::shared_ptr<SimEnvWorldStateSpace> SimEnvWorldStateSpacePtr;
                typedef std::shared_ptr<const SimEnvWorldStateSpace> SimEnvWorldStateSpaceConstPtr;
                typedef std::weak_ptr<SimEnvWorldStateSpace> SimEnvWorldStateSpaceWeakPtr;
                typedef std::weak_ptr<const SimEnvWorldStateSpace> SimEnvWorldStateSpaceConstWeakPtr;

                namespace internal {
                    /**
                     * Enum to identifiy types of subspaces - for internal use only.
                     */
                    enum struct SubspaceType {
                        RealVector, SO3, SO2
                    };

                    /**
                     * Struct to identify type and dimension of a subspace - for internal use only.
                     */
                    struct SubstateTypeDescription {
                        SubspaceType type;
                        unsigned int dim;
                        SubstateTypeDescription() {}
                        SubstateTypeDescription(SubspaceType type, unsigned int dim) :
                                type(type), dim(dim) {}
                    };

                }

                /**
                 * Configuration space for all active dofs of a sim_env::Object
                 */
                class SimEnvObjectConfigurationSpace : public ::ompl::base::CompoundStateSpace {
                public:
                    class StateType : public ::ompl::base::CompoundStateSpace::StateType {
                    public:
                        StateType(const std::vector<internal::SubstateTypeDescription>& desc, unsigned int dimension);
                        StateType() = delete;
                        ~StateType();
                        Eigen::VectorXf getConfiguration() const;
                        void getConfiguration(Eigen::VectorXf& vec) const;
                        void setConfiguration(const Eigen::VectorXf& config);
                    private:
                        const std::vector<internal::SubstateTypeDescription> _state_descriptions;
                        unsigned int _dimension;
                    };

                    SimEnvObjectConfigurationSpace(sim_env::ObjectConstPtr object,
                                                   const Eigen::MatrixX2f& position_limits,
                                                   double pos_weight,
                                                   double orientation_weight,
                                                   double joint_weight);
                    ~SimEnvObjectConfigurationSpace();

                    /** Overrides from CompoundStateSpace */
                    ::ompl::base::State* allocState() const override;
                    void freeState(::ompl::base::State* state) const override;

                private:
                    sim_env::ObjectConstWeakPtr _object;
                    std::vector<internal::SubstateTypeDescription> _state_descriptions;
                    const Eigen::VectorXi _active_dofs;
                    Eigen::MatrixX2f _limits;
                    double _position_weight;
                    double _orientation_weight;
                    double _joint_weight;
                    const std::string _log_prefix;

                    void addPoseSubspace(sim_env::ObjectConstPtr object);
                    void addPoseSubspace2D(sim_env::ObjectConstPtr object);
                    void addPoseSubspace3D(sim_env::ObjectConstPtr object);
                    void addJointsSubspace(sim_env::ObjectConstPtr object);
                    // arguments: dof = _active_dofs[idx] if dof != -1
                    void addSingleBasePoseSpace2D(int dof, unsigned int idx);
                    void addSingleBasePoseSpace3D(int dof, unsigned int idx);
                    sim_env::LoggerConstPtr getLogger() const;
                    ::ompl::base::RealVectorBounds makeBounds(const std::vector<unsigned int>& indices);
                };
                typedef SimEnvObjectConfigurationSpace::StateType SimEnvObjectConfiguration;

                /**
                 * SimEnvObjectVelocitySpace encompasses the velocities of all active degrees of freedom
                 * of a sim_env::Object.
                 */
                class SimEnvObjectVelocitySpace : public ::ompl::base::CompoundStateSpace {
                public:

                    class StateType : public ::ompl::base::CompoundStateSpace::StateType {
                    public:
                        StateType(const std::vector<internal::SubstateTypeDescription>& desc, unsigned int dimension);
                        StateType() = delete;
                        ~StateType();
                        Eigen::VectorXf getVelocity() const;
                        void getVelocity(Eigen::VectorXf& vec) const;
                        void setVelocity(const Eigen::VectorXf& vel);
                    private:
                        const std::vector<internal::SubstateTypeDescription> _state_descriptions;
                        unsigned int _dimension;
                    };

                    SimEnvObjectVelocitySpace(sim_env::ObjectConstPtr object,
                                              const Eigen::MatrixX2f& velocity_limits,
                                              double pos_weight,
                                              double orientation_weight,
                                              double joint_weight);
                    ~SimEnvObjectVelocitySpace();

                    /** Overrides from CompoundStateSpace */
                    ::ompl::base::State* allocState() const override;
                    void freeState(::ompl::base::State* state) const override;

                private:
                    sim_env::ObjectConstWeakPtr _object;
                    std::vector<internal::SubstateTypeDescription> _state_descriptions;
                    const Eigen::VectorXi _active_dofs;
                    Eigen::MatrixX2f _limits;
                    double _position_vel_weight;
                    double _orientation_vel_weight;
                    double _joint_vel_weight;
                    const std::string _log_prefix;

                    void addPoseVelocitySubspace(sim_env::ObjectConstPtr object);
                    void addJointsVelocitySubspace(sim_env::ObjectConstPtr object);
                    sim_env::LoggerConstPtr getLogger() const;
                    ::ompl::base::RealVectorBounds makeBounds(const std::vector<unsigned int>& indices);
                };
                typedef SimEnvObjectVelocitySpace::StateType SimEnvObjectVelocity;

                /**
                 * This class implements a state space for a sim_env object.
                 * The state space is defined through the active degrees of freedom of the object
                 * at the point of creation. A boolean passed on construction determines whether
                 * the state space entails velocities or not.
                 */
                class SimEnvObjectStateSpace : public ::ompl::base::CompoundStateSpace {
                public:
                    /**
                     * Distance weights used in the distance function implemented by ::ompl::base::CompoundStateSpace
                     */
                    struct DistanceWeights {
                        double position_weight;
                        double orientation_weight;
                        double joint_weight;
                        double configuration_weight;
                        double velocity_weight;

                        /**
                         * The default constructor assigns some default values.
                         */
                        DistanceWeights() {
                            position_weight = 1.0;
                            orientation_weight = 0.5;
                            joint_weight = 0.5;
                            configuration_weight = 1.0;
                            velocity_weight = 0.1;
                        }
                    };

                    class StateType : public ::ompl::base::CompoundStateSpace::StateType {
                    public:
                        StateType(int num_dofs, bool configuration_only=true);
                        StateType() = delete;
                        ~StateType();
                        Eigen::VectorXf getConfiguration() const;
                        void getConfiguration(Eigen::VectorXf& vec) const;
                        void setConfiguration(const Eigen::VectorXf& vec);
                        bool hasVelocity() const;
                        /**
                         * Returns the velocity of the object in this state.
                         * Velocities are always zero if this is only a configuration.
                         * @return
                         */
                        Eigen::VectorXf getVelocity() const;
                        void getVelocity(Eigen::VectorXf& vec) const;
                        void setVelocity(const Eigen::VectorXf& vec);
                    private:
                        bool _configuration_only;
                        int _num_dofs;
                    };

                    /**
                     * Creates a new object state space for a sim_env::Object based on the currently active
                     * degrees of freedom. This may also be a sim_env::Robot. All dofs are limited in position
                     * and velocity by the limits provided by object. Optionally, different position
                     * and velocity limits can be imposed by providing the respective arguments.
                     *
                     * @param object - the object/robot to create the state space for
                     * @param position_only - (optional) flag on whether to include velocities in the state space
                     * @param position_limits - (optional) position limits for all active dofs.
                     * @param velocity_limits - (optional) velocity limits for all active dofs (ignored if position_only=true)
                     * @param weights - (optional) a struct containing weights for the distance function
                     */
                    SimEnvObjectStateSpace(sim_env::ObjectConstPtr object,
                                           bool position_only=true,
                                           const Eigen::MatrixX2f& position_limits=Eigen::MatrixX2f(),
                                           const Eigen::MatrixX2f& velocity_limits=Eigen::MatrixX2f(),
                                           const DistanceWeights& weights=DistanceWeights());

                    ~SimEnvObjectStateSpace();

                    /** Overrides from CompoundStateSpace */
                    ::ompl::base::State* allocState() const override;
                    void freeState(::ompl::base::State* state) const override;

                private:
                    sim_env::ObjectConstWeakPtr _object;
                    bool _position_only;
                    const Eigen::VectorXi _active_dofs;
                };
                typedef SimEnvObjectStateSpace::StateType SimEnvObjectState;


                /**
                 * Struct that represents pose and velocity limits for all objects in the planning scene.
                 */
                struct PlanningSceneBounds {
                    Eigen::Array2f x_limits;
                    Eigen::Array2f y_limits;
                    Eigen::Array2f z_limits;
                    float max_velocity;
                    float max_rotation_vel;
                };

                /**
                 * Class that represents the planning state of a world scene. May be
                 * Cartesian product of configuration spaces or of state spaces.
                 */
                class SimEnvWorldStateSpace : public ::ompl::base::CompoundStateSpace {
                public:
                    class StateType : public ::ompl::base::CompoundStateSpace::StateType {
                    public:
                        StateType(unsigned int num_objects);
                        StateType() = delete;
                        ~StateType();

                        unsigned int getNumObjects() const;
                        SimEnvObjectState* getObjectState(unsigned int i) const;

                    private:
                        unsigned int _num_objects;

                    };

                    typedef std::unordered_map<std::string, SimEnvObjectStateSpace::DistanceWeights> WeightMap;

                    /**
                     * Creates a new state space for the provided world. The state space of a world is the
                     * Cartesian product of the state spaces of all active objects/robots in it. An object
                     * is considered active if it has at least one active degree of freedom.
                     * @param world - the world to create the state space for
                     * @param bounds - bounds limiting the base positions of objects to a certain workspace area
                     * @param position_only - if true, the resulting state space is the Cartesian product of
                     *                      configuration spaces, i.e. does not include any velocities. Otherwise the
                     *                      state spaces include velocities.
                     * @param weight - a map that maps object/robot names to SimEnvObjectStateSpace::DistanceWeight instances.
                     *              If there is no DistanceWeight for an object/robot in the map, the default DistanceWeight is used.
                     */
                    SimEnvWorldStateSpace(sim_env::WorldConstPtr world,
                                          const PlanningSceneBounds& bounds,
                                          bool position_only=true,
                                          const WeightMap& weight=WeightMap());
                    ~SimEnvWorldStateSpace();

                    sim_env::ObjectConstPtr getObject(unsigned int i) const;
                    std::string getObjectName(unsigned int i) const;
                    unsigned int getNumObjects() const;

                    /** Overrides from CompoundStateSpace */
                    ::ompl::base::State* allocState() const override;
                    void freeState(::ompl::base::State* state) const override;

                private:
                    sim_env::WorldConstWeakPtr _world;
                    std::unordered_map<std::string, std::shared_ptr<SimEnvObjectStateSpace> > _state_space_map;
                    std::vector<std::string> _object_names;

                    void constructLimits(sim_env::ObjectConstPtr object, const PlanningSceneBounds& bounds,
                                         Eigen::ArrayX2f& position_limits, Eigen::ArrayX2f& velocity_limits) const;
                };
                typedef SimEnvWorldStateSpace::StateType SimEnvWorldState;

                /**
                 * A state validity checker for SimEnvWorldStates.
                 */
                class SimEnvValidityChecker : public ::ompl::base::StateValidityChecker {
                public:
                    /**
                     * The collision policy determines what kind of collisions are allowed, i.e. are considered
                     * valid. By default, all collisions are allowed.
                     */
                    class CollisionPolicy {
                        // TODO implement this properly, at the moment everything is hardcoded to be always allowed
                    public:
                        void setStaticCollisions(bool allowed);
                        bool staticCollisionsAllowed() const;
                        void setCollision(const std::string& obj1, const std::string& obj2, bool allowed);
                        void setCollision(sim_env::ObjectConstPtr obj1, sim_env::ObjectConstPtr obj2, bool allowed);
                        bool collisionAllowed(const std::string& obj1, const std::string& obj2) const;
                        bool collisionAllowed(sim_env::ObjectConstPtr obj1, sim_env::ObjectConstPtr obj2) const;
                    };

                    SimEnvValidityChecker(::ompl::base::SpaceInformationPtr si, sim_env::WorldPtr world);
                    ~SimEnvValidityChecker();

                    /**
                     * Checks whether a state is valid or not.
                     * A state is valid, if it is physically feasible (i.e. not overlaps between rigid bodies)
                     * and within bounds. Optionally, collisions between certain objects may be prohibited.
                     * @param state - must be an instance of SimEnvWorldStateSpace
                     * @return
                     */
                    bool isValid(const ::ompl::base::State* state) const override;

                    bool checkContact(const sim_env::Contact&) const;
                    /**
                     * Public access to collision policy
                     */
                    CollisionPolicy collision_policy;
                private:
                    sim_env::WorldPtr _world;
                    SimEnvWorldStateSpaceConstWeakPtr _world_space;
                };

            }
        }
    }
}
#endif //MANIPULATION_PLANNING_SUITE_SIMENVSTATE_H
