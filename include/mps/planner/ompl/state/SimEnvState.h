//
// Created by joshua on 8/14/17.
//

#ifndef MANIPULATION_PLANNING_SUITE_SIMENVSTATE_H
#define MANIPULATION_PLANNING_SUITE_SIMENVSTATE_H

// STL includes
#include <sstream>
#include <memory>
#include <string>
#include <unordered_map>
// Eigen includes
#include <Eigen/Core>
// ompl includes
#include <ompl/base/StateSpace.h>
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

                    /**
                     * Create name for a state space for the specified dofs.
                     * @param dofs - degrees of freedom the state space is for
                     * @param type_name - type description of state space
                     * @return a name for the state space
                     */
                    std::string createSpaceName(const std::vector<int>& dofs, const std::string& type_name) {
                        std::stringstream ss;
                        ss << "DOFs[";
                        for (auto& dof : dofs) {
                            ss << dof << ",";
                        }
                        ss << "]-" << type_name;
                        return ss.str();
                    }
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
                    double _position_weight;
                    double _orientation_weight;
                    double _joint_weight;

                    void addPoseSubspace(sim_env::ObjectConstPtr object);
                    void addJointsSubspace(sim_env::ObjectConstPtr object);
                    void addSingleBasePoseSpace2D(int dof_idx);
                    void addSingleBasePoseSpace3D(int dof_idx);
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
                    double _position_vel_weight;
                    double _orientation_vel_weight;
                    double _joint_vel_weight;

                    void addPoseVelocitySubspace(sim_env::ObjectConstPtr object);
                    void addJointsVelocitySubspace(sim_env::ObjectConstPtr object);
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
                    struct DistanceWeights {
                        double position_weight;
                        double orientation_weight;
                        double joint_weight;
                        double configuration_weight;
                        double velocity_weight;

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
                        StateType(bool configuration_only=true);
                        StateType() = delete;
                        ~StateType();
                        Eigen::VectorXf getConfiguration() const;
                        void getConfiguration(Eigen::VectorXf& vec) const;
                        void setConfiguration(const Eigen::VectorXf& vec);
                        bool hasVelocity() const;
                        Eigen::VectorXf getVelocity() const;
                        void getVelocity(Eigen::VectorXf& vec) const;
                        void setVelocity(const Eigen::VectorXf& vec);
                    private:
                        bool _configuration_only;
                    };

                    SimEnvObjectStateSpace(sim_env::ObjectConstPtr object, const DistanceWeights& weights=DistanceWeights(),
                                           bool position_only=true);
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

                    SimEnvWorldStateSpace(sim_env::WorldConstPtr world);
                    ~SimEnvWorldStateSpace();

                    sim_env::ObjectConstPtr getObject(unsigned int i) const;
                    unsigned int getNumObjects() const;

                    /** Overrides from CompoundStateSpace */
                    ::ompl::base::State* allocState() const override;
                    void freeState(::ompl::base::State* state) const override;

                private:
                    sim_env::WorldConstWeakPtr _world;
                    std::unordered_map<std::string, std::shared_ptr<SimEnvObjectStateSpace> > _state_space_map;
                    std::vector<std::string> _object_names;
                };
                typedef SimEnvWorldStateSpace::StateType SimEnvWorldState;

                class SimEnvValidityChecker {
                    // TODO State validity checker using SimEnv
                };

                class SimEnvStatePropagator {
                    // TODO implement state propagator using sim_env. The state propagator operates on VelocityControls
                };
            }
        }
    }
}
#endif //MANIPULATION_PLANNING_SUITE_SIMENVSTATE_H
