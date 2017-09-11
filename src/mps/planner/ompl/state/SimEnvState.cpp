//
// Created by joshua on 8/16/17.
//
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <sim_env/utils/EigenUtils.h>
#include <mps/planner/ompl/state/SimEnvState.h>
#include <mps/planner/util/Serialize.h>
#include <mps/planner/util/Logging.h>
#include <mps/planner/util/Math.h>
#include <boost/format.hpp>
#include <sstream>
#include <string>
#include <boost/math/constants/constants.hpp>

namespace mps {
    namespace planner {
        namespace ompl {
            namespace state {
                namespace internal {
                    /**
                     * Create name for a state space for the specified dofs.
                     * @param dofs - degrees of freedom the state space is for
                     * @param type_name - type description of state space
                     * @return a name for the state space
                     */
                    std::string createSpaceName(const std::vector<int> &dofs, const std::string &type_name) {
                        std::stringstream ss;
                        ss << "DOFs[";
                        for (auto& dof : dofs) {
                            ss << dof << ",";
                        }
                        ss << "]-" << type_name;
                        return ss.str();
                    }
                }
            }
        }
    }
}

typedef boost::format bf;
namespace mps_logging = mps::planner::util::logging;
using namespace mps::planner::ompl::state;
////////////////////////////////////////////////////////////////////////////////////////////
/////////////////// SimEnvObjectConfigurationSpace::StateType aka SimEnvConfiguration //////
////////////////////////////////////////////////////////////////////////////////////////////
SimEnvObjectConfigurationSpace::StateType::StateType(const std::vector<internal::SubstateTypeDescription>& desc,
                                                     unsigned int dimension) :
        _state_descriptions(desc),
        _dimension(dimension)
{
}

SimEnvObjectConfigurationSpace::StateType::~StateType() {
}

Eigen::VectorXf SimEnvObjectConfigurationSpace::StateType::getConfiguration() const {
    Eigen::VectorXf config;
    getConfiguration(config);
    return config;
}

void SimEnvObjectConfigurationSpace::StateType::getConfiguration(Eigen::VectorXf& config) const {
    config.resize(_dimension);
    unsigned int dof_idx = 0;
    for (size_t space_idx = 0; space_idx < _state_descriptions.size(); ++space_idx) {
        switch (_state_descriptions.at(space_idx).type ) {
            case internal::SubspaceType::RealVector:
            {
                auto * real_vector_state =
                        static_cast<::ompl::base::RealVectorStateSpace::StateType*>(components[space_idx]);
                for (unsigned int i = 0; i < _state_descriptions.at(space_idx).dim; ++i) {
                    config[dof_idx] = (float) real_vector_state->values[i];
                    dof_idx++;
                }
                break;
            }
            case internal::SubspaceType::SO3:
            {
                auto * so3_state =
                        static_cast<::ompl::base::SO3StateSpace::StateType*>(components[space_idx]);
                Eigen::Vector3f euler;
                Eigen::Quaternionf quat((const float &) so3_state->w, (const float &) so3_state->x,
                                        (const float &) so3_state->y, (const float &) so3_state->z);
                sim_env::utils::eigen::quaternionToEuler(quat, euler);
                config[dof_idx++] = euler[0];
                config[dof_idx++] = euler[1];
                config[dof_idx++] = euler[2];
                break;
            }
            case internal::SubspaceType ::SO2:
            {
                auto * so2_state =
                        static_cast<::ompl::base::SO2StateSpace::StateType*>(components[space_idx]);
                config[dof_idx++] = (float)so2_state->value;
                break;
            }
        }
    }
}

void SimEnvObjectConfigurationSpace::StateType::setConfiguration(const Eigen::VectorXf& config) {
    unsigned int dof_idx = 0;
    for (size_t space_idx = 0; space_idx < _state_descriptions.size(); ++space_idx) {
        switch (_state_descriptions.at(space_idx).type ) {
            case internal::SubspaceType::RealVector:
            {
                auto * real_vector_state =
                        static_cast<::ompl::base::RealVectorStateSpace::StateType*>(components[space_idx]);
                for (unsigned int i = 0; i < _state_descriptions.at(space_idx).dim; ++i) {
                    real_vector_state->values[i] = (double) config[dof_idx++];
                }
                break;
            }
            case internal::SubspaceType::SO3:
            {
                auto * so3_state =
                        static_cast<::ompl::base::SO3StateSpace::StateType*>(components[space_idx]);
                Eigen::Quaternionf quat;
                sim_env::utils::eigen::eulerToQuaternion(config[dof_idx],
                                                         config[dof_idx + 1],
                                                         config[dof_idx + 2],
                                                         quat);
                dof_idx += 3;
                so3_state->w = quat.w();
                so3_state->x = quat.x();
                so3_state->y = quat.y();
                so3_state->z = quat.z();
                break;
            }
            case internal::SubspaceType ::SO2:
            {
                auto * so2_state =
                        static_cast<::ompl::base::SO2StateSpace::StateType*>(components[space_idx]);
                so2_state->value = (double)config[dof_idx++];
                util::math::normalize_orientation(so2_state->value);
                break;
            }
        }
    }
}

void SimEnvObjectConfigurationSpace::StateType::serializeInNumbers(std::ostream &ostream) const {
    ostream << getConfiguration().transpose().format(eigen_format);
}

////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////// SimEnvObjectConfigurationSpace ////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
SimEnvObjectConfigurationSpace::SimEnvObjectConfigurationSpace(sim_env::ObjectConstPtr object,
                                                               const Eigen::MatrixX2f& position_limits,
                                                               double pos_weight,
                                                               double orientation_weight,
                                                               double joint_weight) :
    _object(object),
    _active_dofs(object->getActiveDOFs()),
    _position_weight(pos_weight),
    _orientation_weight(orientation_weight),
    _joint_weight(joint_weight),
    _log_prefix("[mps::planner::ompl::state::SimEnvObjectConfigurationSpace::")
{
    static const std::string log_prefix(_log_prefix + "SimEnvObjectConfigurationSpace]");
    assert(_active_dofs.size() > 0);
    setName("sim_env::Object(" + object->getName() + ")-ConfigurationSpace");
    _limits = position_limits;
    if (_limits.rows() == 0) {
        _limits = object->getDOFPositionLimits();
    }
    addPoseSubspace(object);
    addJointsSubspace(object);
    lock();
    mps_logging::logDebug(bf("Created configurations space for object %s") % object->getName(),
                          log_prefix);
    mps_logging::logDebug(bf("The dimension of this space is %i") % getDimension(),
                          log_prefix);
    mps_logging::logDebug(bf("The bounds are\n %d") % _limits,
                          log_prefix);
}

SimEnvObjectConfigurationSpace::~SimEnvObjectConfigurationSpace() = default;

::ompl::base::State* SimEnvObjectConfigurationSpace::allocState() const {
    SimEnvObjectConfiguration* configuration = new SimEnvObjectConfiguration(_state_descriptions,
                                                                            _active_dofs.size());
    allocStateComponents(configuration);
    return configuration;
}

void SimEnvObjectConfigurationSpace::freeState(::ompl::base::State* state) const {
    SimEnvObjectConfiguration* configuration = static_cast<SimEnvObjectConfiguration*>(state);
    for (unsigned int i = 0; i < componentCount_; ++i) {
        components_[i]->freeState(configuration->components[i]);
    }
    delete[] configuration->components;
    delete configuration;
}

//void SimEnvObjectConfigurationSpace::computeDirection(StateType const* state_1,
//                                                      StateType const* state_2,
//                                                      Eigen::VectorXf& dir) const {
//    Eigen::VectorXf a = state_1->getConfiguration();
//    Eigen::VectorXf b = state_2->getConfiguration(),
//    computeDirection()
//    computeDirection(a, b, dir);
//}

void SimEnvObjectConfigurationSpace::computeDirection(const Eigen::VectorXf& config_1,
                                                      const Eigen::VectorXf& config_2,
                                                      Eigen::VectorXf& dir) const {
    assert(config_1.size() == config_2.size());
    dir.resize(config_1.size());
    unsigned int c_idx = 0;
    for (const auto& space_desc : _state_descriptions) {
        switch(space_desc.type) {
            case internal::SubspaceType::RealVector:
            {
                dir.segment(c_idx, space_desc.dim) = config_2.segment(c_idx, space_desc.dim)
                                                     - config_1.segment(c_idx, space_desc.dim);
                break;
            }
            case internal::SubspaceType::SO2:
            {
                dir[c_idx] = mps::planner::util::math::shortest_direction_so2(config_1[c_idx], config_2[c_idx]);
                break;
            }
            case internal::SubspaceType::SO3:
            {
                for (unsigned int i = 0; i < space_desc.dim; ++i) {
                    dir[c_idx + i] = mps::planner::util::math::shortest_direction_so2(
                            config_1[c_idx + i], config_2[c_idx + i]);
                }
                break;
            }
        }
        c_idx += space_desc.dim;
    }
}

void SimEnvObjectConfigurationSpace::addPoseSubspace(sim_env::ObjectConstPtr object) {
    if (object->isStatic()) { // no pose subspace to add
        return;
    }
    // if the object is not static we need to distinguish two different cases
    // either we are in a 2d world (getNumBaseDOFs == 3) or in a 3d world (getNumBaseDOFs == 6)
    if (object->getNumBaseDOFs() == 3) { // we are in a 2d world
        addPoseSubspace2D(object);
    } else {
        assert(object->getNumBaseDOFs() == 6); // we are in a 3d world
        addPoseSubspace3D(object);
    }
}

void SimEnvObjectConfigurationSpace::addPoseSubspace2D(sim_env::ObjectConstPtr object) {
    sim_env::LoggerConstPtr logger = object->getConstWorld()->getConstLogger();
    // let's first create a helper vector for which idx_helper[i] = _active_dofs[i] if i < _active_dofs.size(),
    // else idx_helper[i] = -1 for i = 0 .. 2
    Eigen::VectorXi idx_helper(3);
    for (unsigned int i = 0; i < idx_helper.size(); ++i) {
        idx_helper[i] = _active_dofs.size() > i ? _active_dofs[i] : -1;
    }
    if (idx_helper[0] == 0 and idx_helper[1] == 1) {
        // we have both x and y, so let's add a 2d real vector space
        auto pos_state_space = std::make_shared<::ompl::base::RealVectorStateSpace>(2);
        pos_state_space->setBounds(makeBounds({0, 1}));
        pos_state_space->setName(internal::createSpaceName({0,1}, "ConfigurationSpace"));
        addSubspace(pos_state_space, _position_weight);
        internal::SubstateTypeDescription desc(internal::SubspaceType::RealVector, 2);
        _state_descriptions.push_back(desc);
        addSingleBasePoseSpace2D(idx_helper[2], 2); // and maybe also SO
    } else {
        // else we can treat each potential base dof independently
        addSingleBasePoseSpace2D(idx_helper[0], 0); // might be position or orientation
        addSingleBasePoseSpace2D(idx_helper[1], 1); // might be orientation
    }

}

void SimEnvObjectConfigurationSpace::addPoseSubspace3D(sim_env::ObjectConstPtr object) {
    sim_env::LoggerConstPtr logger = object->getConstWorld()->getConstLogger();
    // let's first create a helper vector for which idx_helper[i] = _active_dofs[i] if i < _active_dofs.size(),
    // else idx_helper[i] = -1 for i = 0 .. 5
    Eigen::VectorXi idx_helper(6);
    for (unsigned int i = 0; i < idx_helper.size(); ++i) {
        idx_helper[i] = _active_dofs.size() > i ? _active_dofs[i] : -1;
    }
    unsigned int rdio = 0; // rotation dof index offset

    if (idx_helper[0] == 0 and idx_helper[1] == 1 and idx_helper[2] == 2) {
        // all position dimensions are part of the configspace
        auto pos_state_space = std::make_shared<::ompl::base::RealVectorStateSpace>(3);
        pos_state_space->setName(internal::createSpaceName({0,1,2}, "ConfigurationSpace"));
        pos_state_space->setBounds(makeBounds({0,1,2}));
        addSubspace(pos_state_space, _position_weight);
        internal::SubstateTypeDescription desc(internal::SubspaceType::RealVector, 3);
        _state_descriptions.push_back(desc);
        rdio = 3;
    } else if ((idx_helper[0] == 0 and idx_helper[1] == 1) or
               (idx_helper[0] == 0 and idx_helper[1] == 2) or
               (idx_helper[0] == 1 and idx_helper[1] == 2)) {
        // only 2 position dimensions are part of the configspace
        auto pos_state_space = std::make_shared<::ompl::base::RealVectorStateSpace>(2);
        pos_state_space->setName(internal::createSpaceName({idx_helper[0], idx_helper[1]}, "ConfigurationSpace"));
        pos_state_space->setBounds(makeBounds({0, 1}));
        addSubspace(pos_state_space, _position_weight);
        internal::SubstateTypeDescription desc(internal::SubspaceType::RealVector, 2);
        _state_descriptions.push_back(desc);
        rdio = 2;
    } else {
        // at most one dimension is part of the config space (active_dofs should be sorted)
        addSingleBasePoseSpace3D(idx_helper[0], 0);
        rdio = (unsigned int)(idx_helper[0] == 0 || idx_helper[0] == 1 || idx_helper[0] == 2);
    }
    // now do a similiar thing for the orientation
    if (idx_helper[rdio] == 3 and idx_helper[rdio + 1] == 4 and idx_helper[rdio + 2] == 5) {
        // we have all rotational degrees of freedom
        ::ompl::base::StateSpacePtr orientation_state_space = std::make_shared<::ompl::base::SO3StateSpace>();
        orientation_state_space->setName(internal::createSpaceName({3,4,5}, "ConfigurationSpace"));
        addSubspace(orientation_state_space, _orientation_weight);
        internal::SubstateTypeDescription desc(internal::SubspaceType::SO3, 3);
        _state_descriptions.push_back(desc);
    } else {
        // we have at most 2 rotational dofs
        // TODO in case we still have two rotational degrees of freedom, adding them as separate spaces
        // TODO might have undesirable consequences for the distance function
        addSingleBasePoseSpace3D(idx_helper[rdio], rdio);
        addSingleBasePoseSpace3D(idx_helper[rdio + 1], rdio + 1);
    }
}

void SimEnvObjectConfigurationSpace::addJointsSubspace(sim_env::ObjectConstPtr object) {
    std::vector<int> joint_idx;
    std::vector<unsigned int> indices;
    for (unsigned int i = 0; i < _active_dofs.size(); ++i) {
        int dof = _active_dofs[i];
        if (dof >= object->getNumBaseDOFs()) {
            joint_idx.push_back(dof);
            indices.push_back(i);
        }
    }
    if (joint_idx.empty()) return;
    auto space = std::make_shared<::ompl::base::RealVectorStateSpace>(joint_idx.size());
    space->setName(internal::createSpaceName(joint_idx, "ConfigurationSpace"));
    space->setBounds(makeBounds(indices));
    addSubspace(space, _joint_weight);
    _state_descriptions.emplace_back(internal::SubstateTypeDescription(internal::SubspaceType::RealVector,
                                                                    (unsigned int) joint_idx.size()));
}

void SimEnvObjectConfigurationSpace::addSingleBasePoseSpace2D(int dof, unsigned int idx) {
    sim_env::LoggerConstPtr logger = getLogger();
    ::ompl::base::StateSpacePtr space;
    internal::SubstateTypeDescription type_desc;
    type_desc.dim = 1;
    double weight = 1.0;
    if (dof == 0 or dof == 1) { // check whether its a base position
        assert(_active_dofs[idx] == dof);
        auto real_space = std::make_shared<::ompl::base::RealVectorStateSpace>(1);
        real_space->setBounds(makeBounds({idx}));
        space = real_space;
        type_desc.type = internal::SubspaceType::RealVector;
        weight = _position_weight;
    } else if (dof == 2) { // or base orientation
        assert(_active_dofs[idx] == dof);
        space = std::make_shared<::ompl::base::SO2StateSpace>();
        weight = _orientation_weight;
        type_desc.type = internal::SubspaceType::SO2;
    } else {
        return; // or some other odf, then we don't need to add it
    }
    space->setName(internal::createSpaceName({dof}, "ConfigurationSpace"));
    addSubspace(space, weight);
    _state_descriptions.push_back(type_desc);
}

void SimEnvObjectConfigurationSpace::addSingleBasePoseSpace3D(int dof, unsigned int idx) {
    sim_env::LoggerConstPtr logger = getLogger();
    ::ompl::base::StateSpacePtr space;
    double weight = 1.0;
    internal::SubstateTypeDescription type_desc;
    type_desc.dim = 1;
    if (dof == 0 or dof == 1 or dof == 2) { // check whether it's a base position
        assert(_active_dofs[idx] == dof);
        auto real_space = std::make_shared<::ompl::base::RealVectorStateSpace>(1);
        real_space->setBounds(makeBounds({idx}));
        space = real_space;
        weight = _position_weight;
        type_desc.type = internal::SubspaceType::RealVector;
    } else if (dof == 3 or dof == 4 or dof == 5) { // else it's a base orientation
        assert(_active_dofs[idx] == dof);
        space = std::make_shared<::ompl::base::SO2StateSpace>();
        weight = _orientation_weight;
        type_desc.type = internal::SubspaceType::SO2;
    } else {
        return; // do not proceed further
    }
    space->setName(internal::createSpaceName({dof}, "ConfigurationSpace"));
    addSubspace(space, weight);
    _state_descriptions.push_back(type_desc);
}

sim_env::LoggerConstPtr SimEnvObjectConfigurationSpace::getLogger() const {
    auto object = _object.lock();
    if (!object) {
        throw std::logic_error("[mps::planner::ompl::state::SimEnvObjectConfigurationSpace::getLogger]"
                                       "Could not acquire access to sim_env::Object. This state space should not exist anymore.");
    }
    return object->getConstWorld()->getConstLogger();
}

::ompl::base::RealVectorBounds SimEnvObjectConfigurationSpace::makeBounds(const std::vector<unsigned int> &indices) {
    unsigned int dim = (unsigned int) indices.size();
    ::ompl::base::RealVectorBounds bounds(dim);
    for (unsigned int i = 0; i < dim; ++i) {
        bounds.setLow(i, _limits(indices[i], 0));
        bounds.setHigh(i, _limits(indices[i], 1));
    }
    return bounds;
}

////////////////////////////////////////////////////////////////////////////////////////
/////////////////// SimEnvObjectVelocity aka SimEnvObjectVelocitySpace::StateType //////
////////////////////////////////////////////////////////////////////////////////////////
SimEnvObjectVelocitySpace::StateType::StateType(const std::vector<internal::SubstateTypeDescription>& desc,
                                                unsigned int dimension) :
        _state_descriptions(desc),
        _dimension(dimension)
{
}

SimEnvObjectVelocitySpace::StateType::~StateType() {
}

Eigen::VectorXf SimEnvObjectVelocitySpace::StateType::getVelocity() const {
    Eigen::VectorXf vel;
    getVelocity(vel);
    return vel;
}

void SimEnvObjectVelocitySpace::StateType::getVelocity(Eigen::VectorXf& vel) const {
    vel.resize(_dimension);
    unsigned int dof_idx = 0;
    for (size_t space_idx = 0; space_idx < _state_descriptions.size(); ++space_idx) {
        assert(_state_descriptions.at(space_idx).type == internal::SubspaceType::RealVector);
        ::ompl::base::RealVectorStateSpace::StateType* real_vector_state =
                static_cast<::ompl::base::RealVectorStateSpace::StateType*>(components[space_idx]);
        for (unsigned int i = 0; i < _state_descriptions.at(space_idx).dim; ++i) {
            vel[dof_idx] = (float) real_vector_state->values[i];
            dof_idx++;
        }
    }
}

void SimEnvObjectVelocitySpace::StateType::setVelocity(const Eigen::VectorXf& vel) {
    unsigned int dof_idx = 0;
    for (size_t space_idx = 0; space_idx < _state_descriptions.size(); ++space_idx) {
        assert(_state_descriptions.at(space_idx).type == internal::SubspaceType::RealVector);
        ::ompl::base::RealVectorStateSpace::StateType* real_vector_state =
                static_cast<::ompl::base::RealVectorStateSpace::StateType*>(components[space_idx]);
        for (unsigned int i = 0; i < _state_descriptions.at(space_idx).dim; ++i) {
            real_vector_state->values[i] = vel[dof_idx];
            dof_idx++;
        }
    }
    assert(dof_idx == _dimension - 1);
}

void SimEnvObjectVelocitySpace::StateType::serializeInNumbers(std::ostream& ostream) const {
    ostream << getVelocity().transpose().format(eigen_format);
}


////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////// SimEnvObjectVelocitySpace ////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
SimEnvObjectVelocitySpace::SimEnvObjectVelocitySpace(sim_env::ObjectConstPtr object,
                                                     const Eigen::MatrixX2f& velocity_limits,
                                                     double pos_weight,
                                                     double orientation_weight,
                                                     double joint_weight) :
    _object(object),
    _active_dofs(object->getActiveDOFs()),
    _position_vel_weight(pos_weight),
    _orientation_vel_weight(orientation_weight),
    _joint_vel_weight(joint_weight),
    _log_prefix("[mps::planner::ompl::state::SimEnvObjectVelocitySpace::")
{
    static const std::string log_prefix(_log_prefix + "SimEnvObjectVelocitySpace]");
    assert(_active_dofs.size() > 0);
    setName("sim_env::Object(" + object->getName() + ")-VelocitySpace");
    if (velocity_limits.rows() == 0) {
        _limits = object->getDOFVelocityLimits();
    } else {
        _limits = velocity_limits;
    }
    addPoseVelocitySubspace(object);
    addJointsVelocitySubspace(object);
    lock();
    mps_logging::logDebug(bf("Created velocity space for object %s") % object->getName(),
                          log_prefix);
    mps_logging::logDebug(bf("The dimension of this space is %i") % getDimension(),
                          log_prefix);
    mps_logging::logDebug(bf("The bounds are\n %d") % _limits,
                          log_prefix);
}

SimEnvObjectVelocitySpace::~SimEnvObjectVelocitySpace() = default;

::ompl::base::State* SimEnvObjectVelocitySpace::allocState() const {
    auto* velocity = new SimEnvObjectVelocity(_state_descriptions, _active_dofs.size());
    allocStateComponents(velocity);
    return velocity;
}

void SimEnvObjectVelocitySpace::freeState(::ompl::base::State* state) const {
    auto* velocity = static_cast<SimEnvObjectVelocity*>(state);
    for (unsigned int i = 0; i < componentCount_; ++i) {
        components_[i]->freeState(velocity->components[i]);
    }
    delete[] velocity->components;
    delete velocity;
}

void SimEnvObjectVelocitySpace::addPoseVelocitySubspace(sim_env::ObjectConstPtr object) {
    if (object->isStatic()) { // no pose velocity subspace to add
        return;
    }
    std::vector<int> position_dofs;
    std::vector<unsigned int> position_dof_indices;
    std::vector<int> orientation_dofs;
    std::vector<unsigned int> orientation_dof_indices;
    sim_env::LoggerConstPtr logger = getLogger();
    // 2d world
    int max_num_position_dofs = 2;
    int max_num_orientation_dofs = 1;
    if (object->getNumBaseDOFs() == 6) { // 3d world
        max_num_position_dofs = 3;
        max_num_orientation_dofs = 3;
    }
    // collect indices
    for (unsigned int i = 0; i < _active_dofs.size(); ++i) {
        if (_active_dofs[i] < max_num_position_dofs) {
            position_dofs.push_back(_active_dofs[i]);
            position_dof_indices.push_back(i);
        }
        if (max_num_position_dofs < _active_dofs[i] and
            _active_dofs[i] < max_num_position_dofs + max_num_orientation_dofs) {
            orientation_dofs.push_back(_active_dofs[i]);
            orientation_dof_indices.push_back(i);
        }
    }
    if (!position_dofs.empty()) {
        // TODO we shouldn't use RealVector spaces here (they are box shaped).
        // TODO Should use a ball shaped space
        auto velocity_space = std::make_shared<::ompl::base::RealVectorStateSpace>(position_dofs.size());
        velocity_space->setName(internal::createSpaceName(position_dofs, "VelocitySpace"));
        velocity_space->setBounds(makeBounds(position_dof_indices));
        addSubspace(velocity_space, _position_vel_weight);
        _state_descriptions.emplace_back(internal::SubstateTypeDescription(internal::SubspaceType::RealVector,
                                                                        (unsigned int) position_dofs.size()));
    }
    if (!orientation_dofs.empty()) {
        // TODO we shouldn't use RealVector spaces here (they are box shaped).
        // TODO Should use a ball shaped space
        auto velocity_space = std::make_shared<::ompl::base::RealVectorStateSpace>(orientation_dofs.size());
        velocity_space->setName(internal::createSpaceName(orientation_dofs, "VelocitySpace"));
        velocity_space->setBounds(makeBounds(orientation_dof_indices));
        addSubspace(velocity_space, _orientation_vel_weight);
        _state_descriptions.emplace_back(internal::SubstateTypeDescription(internal::SubspaceType::RealVector,
                                                                        (unsigned int) orientation_dofs.size()));
    }
}

void SimEnvObjectVelocitySpace::addJointsVelocitySubspace(sim_env::ObjectConstPtr object) {
    std::vector<int> joint_idx;
    std::vector<unsigned int> indices;
    for (unsigned int i = 0; i < _active_dofs.size(); ++i) {
        int dof = _active_dofs[i];
        if (dof >= object->getNumBaseDOFs()) {
            joint_idx.push_back(dof);
            indices.push_back(i);
        }
    }
    if (joint_idx.empty()) return;
    auto space = std::make_shared<::ompl::base::RealVectorStateSpace>(joint_idx.size());
    space->setName(internal::createSpaceName(joint_idx, "VelocitySpace"));
    space->setBounds(makeBounds(indices));
    addSubspace(space, _joint_vel_weight);
    _state_descriptions.emplace_back(internal::SubstateTypeDescription(internal::SubspaceType::RealVector,
                                                                    (unsigned int) joint_idx.size()));
}

sim_env::LoggerConstPtr SimEnvObjectVelocitySpace::getLogger() const {
    auto object = _object.lock();
    if (!object) {
        throw std::logic_error("[mps::planner::ompl::state::SimEnvObjectVelocitySpace::getLogger]"
                                       "Could not acquire access to sim_env::Object. This state space should not exist anymore.");
    }
    return object->getConstWorld()->getConstLogger();
}

::ompl::base::RealVectorBounds SimEnvObjectVelocitySpace::makeBounds(const std::vector<unsigned int> &indices) {
    unsigned int dim = (unsigned int) indices.size();
    ::ompl::base::RealVectorBounds bounds(dim);
    for (unsigned int i = 0; i < dim; ++i) {
        bounds.setLow(i, _limits(indices[i], 0));
        bounds.setHigh(i, _limits(indices[i], 1));
    }
    return bounds;
}

////////////////////////////////////////////////////////////////////////////////////////
/////////////////// SimEnvObjectState aka SimEnvObjectState::StateType /////////////////
////////////////////////////////////////////////////////////////////////////////////////
SimEnvObjectStateSpace::StateType::StateType(int num_dofs, bool configuration_only) :
        _configuration_only(configuration_only),
        _num_dofs(num_dofs)
{
}

SimEnvObjectStateSpace::StateType::~StateType() {
}

Eigen::VectorXf SimEnvObjectStateSpace::StateType::getConfiguration() const {
    Eigen::VectorXf config;
    getConfiguration(config);
    return config;
}

void SimEnvObjectStateSpace::StateType::getConfiguration(Eigen::VectorXf& config) const {
    auto* config_component = components[0]->as<SimEnvObjectConfiguration>();
    config_component->getConfiguration(config);
}

void SimEnvObjectStateSpace::StateType::setConfiguration(const Eigen::VectorXf& config) {
    auto* config_component = components[0]->as<SimEnvObjectConfiguration>();
    config_component->setConfiguration(config);
}

bool SimEnvObjectStateSpace::StateType::hasVelocity() const {
    return not _configuration_only;
}

Eigen::VectorXf SimEnvObjectStateSpace::StateType::getVelocity() const {
    Eigen::VectorXf vel;
    getVelocity(vel);
    return vel;
}

void SimEnvObjectStateSpace::StateType::getVelocity(Eigen::VectorXf& vel) const {
    if (not hasVelocity()) {
        vel.resize(_num_dofs);
        vel.setZero();
        return;
    }
    auto* vel_component = components[1]->as<SimEnvObjectVelocity>();
    vel_component->getVelocity(vel);
}

void SimEnvObjectStateSpace::StateType::setVelocity(const Eigen::VectorXf& vel) {
    if (not hasVelocity()) {
        return;
    }
    auto* vel_component = components[1]->as<SimEnvObjectVelocity>();
    vel_component->setVelocity(vel);
}

void SimEnvObjectStateSpace::StateType::serializeInNumbers(std::ostream& ostream) const {
    auto* serializable_component = dynamic_cast<util::serialize::RealValueSerializable*>(components[0]);
    serializable_component->serializeInNumbers(ostream);
    if (hasVelocity()) {
        ostream << ", ";
        serializable_component = dynamic_cast<util::serialize::RealValueSerializable*>(components[1]);
        serializable_component->serializeInNumbers(ostream);
    }
}

void SimEnvObjectStateSpace::StateType::print(std::ostream& ostream) const {
    // TODO do we want to print more here?
    serializeInNumbers(ostream);
}
////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////// SimEnvObjectStateSpace ////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
SimEnvObjectStateSpace::SimEnvObjectStateSpace(sim_env::ObjectConstPtr object,
                                               bool position_only,
                                               const Eigen::MatrixX2f& position_limits,
                                               const Eigen::MatrixX2f& velocity_limits,
                                               const DistanceWeights& weights):
    _object(object),
    _position_only(position_only),
    _active_dofs(object->getActiveDOFs())
{
    assert(_active_dofs.size() > 0);
    setName("sim_env::Object(" + object->getName() + ")-StateSpace");
    ::ompl::base::StateSpacePtr configuration_space =
            std::make_shared<SimEnvObjectConfigurationSpace>(object,
                                                             position_limits,
                                                             weights.position_weight,
                                                             weights.orientation_weight,
                                                             weights.joint_weight);
    addSubspace(configuration_space, weights.configuration_weight);
    if (not _position_only) {
        ::ompl::base::StateSpacePtr velocity_space =
                std::make_shared<SimEnvObjectVelocitySpace>(object,
                                                            velocity_limits,
                                                            weights.position_weight,
                                                            weights.orientation_weight,
                                                            weights.joint_weight);
        addSubspace(velocity_space, weights.velocity_weight);
    }
    lock();
}

SimEnvObjectStateSpace::~SimEnvObjectStateSpace() {
}

SimEnvObjectConfigurationSpacePtr SimEnvObjectStateSpace::getConfigurationSpace() {
    return std::dynamic_pointer_cast<SimEnvObjectConfigurationSpace>(getSubspace(0));
}
SimEnvObjectVelocitySpacePtr SimEnvObjectStateSpace::getVelocitySpace() {
    if (not _position_only) {
        return std::dynamic_pointer_cast<SimEnvObjectVelocitySpace>(getSubspace(1));
    }
    return nullptr;
}

::ompl::base::State* SimEnvObjectStateSpace::allocState() const {
    SimEnvObjectState* object_state = new SimEnvObjectState(_active_dofs.size(), _position_only);
    allocStateComponents(object_state);
    return object_state;
}

void SimEnvObjectStateSpace::freeState(::ompl::base::State* state) const {
    SimEnvObjectState* object_state = static_cast<SimEnvObjectState*>(state);
    for (unsigned int i = 0; i < componentCount_; ++i) {
        components_[i]->freeState(object_state->components[i]);
    }
    delete[] object_state->components;
    delete object_state;
}

////////////////////////////////////////////////////////////////////////////////
//////////// SimEnvWorldState aka SimEnvWorldStateSpace::StateType//////////////
////////////////////////////////////////////////////////////////////////////////
SimEnvWorldStateSpace::StateType::StateType(unsigned int num_objects):
        _num_objects(num_objects)
{
}

SimEnvWorldStateSpace::StateType::~StateType() {
}

unsigned int SimEnvWorldStateSpace::StateType::getNumObjects() const {
    return _num_objects;
}

SimEnvObjectState* SimEnvWorldStateSpace::StateType::getObjectState(unsigned int i) const {
    if (i < _num_objects) {
        return static_cast<SimEnvObjectState*>(components[i]);
    }
    return nullptr;
}

void SimEnvWorldStateSpace::StateType::serializeInNumbers(std::ostream& ostream) const {
    for (unsigned int i = 0; i < getNumObjects(); i++) {
        auto* state = getObjectState(i);
        state->serializeInNumbers(ostream);
        if (i < getNumObjects() - 1) {
            ostream << ", ";
        }
    }
}

void SimEnvWorldStateSpace::StateType::print(std::ostream& ostream) const {
    for (unsigned int i = 0; i < getNumObjects(); i++) {
        auto* state = getObjectState(i);
        state->print(ostream);
        if (i < getNumObjects() - 1) {
            ostream << ", ";
        }
    }
}
////////////////////////////////////////////////////////////////////////////////
////////////////////////////// SimEnvWorldStateSpace ///////////////////////////
////////////////////////////////////////////////////////////////////////////////
SimEnvWorldStateSpace::SimEnvWorldStateSpace(sim_env::WorldConstPtr world,
                                             const PlanningSceneBounds& bounds,
                                             bool position_only,
                                             const WeightMap& weights):
    _world(world)
{
    std::vector<sim_env::ObjectConstPtr> objects;
    world->getObjects(objects, false);
    for (auto& object : objects) {
        if (object->getNumActiveDOFs() > 0) {
            // get limits for this object
            Eigen::ArrayX2f position_limits;
            Eigen::ArrayX2f velocity_limits;
            constructLimits(object, bounds, position_limits, velocity_limits);
            // get distance weights if we have some, else use default
            SimEnvObjectStateSpace::DistanceWeights obj_weight;
            auto weight_iter = weights.find(object->getName());
            if (weight_iter != weights.end()) {
                obj_weight = weight_iter->second;
            }
            SimEnvObjectStateSpacePtr state_space =
                    std::make_shared<SimEnvObjectStateSpace>(object,
                                                             position_only,
                                                             position_limits,
                                                             velocity_limits,
                                                             obj_weight);
            _state_space_map[object->getName()] = state_space;
            addSubspace(state_space, 1.0);
            _object_names.push_back(object->getName());
        }
    }
}

SimEnvWorldStateSpace::~SimEnvWorldStateSpace() {
}

::ompl::base::State* SimEnvWorldStateSpace::allocState() const {
    SimEnvWorldState* world_state = new SimEnvWorldState(_object_names.size());
    allocStateComponents(world_state);
    return world_state;
}

void SimEnvWorldStateSpace::freeState(::ompl::base::State* state) const {
    SimEnvWorldState* world_state = static_cast<SimEnvWorldState*>(state);
    for (unsigned int i = 0; i < componentCount_; ++i) {
        components_[i]->freeState(world_state->components[i]);
    }
    delete[] world_state->components;
    delete world_state;
}

double SimEnvWorldStateSpace::distance(const ::ompl::base::State* state_1, const ::ompl::base::State* state_2) const {
    if (_distance_measure) {
        return _distance_measure->distance(state_1, state_2);
    }
    return ::ompl::base::CompoundStateSpace::distance(state_1, state_2);
}

sim_env::ObjectConstPtr SimEnvWorldStateSpace::getObject(unsigned int i) const {
    if (i >= _object_names.size()) {
        return nullptr;
    }
    sim_env::WorldConstPtr world = _world.lock();
    if (!world) {
        throw std::logic_error("[mps::planner::ompl::state::SimEnvWorldStateSpace::getObject]"
            " The world of this state space does no longer exist.");
    }
    return world->getObjectConst(_object_names[i], false);
}

std::string SimEnvWorldStateSpace::getObjectName(unsigned int i) const {
    if (i >= _object_names.size()) {
        return "";
    }
   return _object_names[i];
}

int SimEnvWorldStateSpace::getObjectIndex(const std::string& obj_name) const {
    for (unsigned int i = 0; i < _object_names.size(); ++i) {
        if (_object_names.at(i) == obj_name) return i;
    }
    return -1;
}

unsigned int SimEnvWorldStateSpace::getNumObjects() const {
    return (unsigned int) _object_names.size();
}

SimEnvObjectStateSpacePtr SimEnvWorldStateSpace::getObjectStateSpace(const std::string& name) {
    int obj_idx = getObjectIndex(name);
    if (obj_idx < 0) {
        return nullptr;
    }
    return std::dynamic_pointer_cast<SimEnvObjectStateSpace>(getSubspace((unsigned int)obj_idx));
}

void SimEnvWorldStateSpace::setToState(sim_env::WorldPtr world, const StateType* state) const {
    for (unsigned int obj_id = 0; obj_id < state->getNumObjects(); ++obj_id) {
        auto* obj_state = state->getObjectState(obj_id);
        std::string obj_name = getObjectName(obj_id);
        sim_env::ObjectPtr obj = world->getObject(obj_name, false);
        obj->setDOFPositions(obj_state->getConfiguration());
        obj->setDOFVelocities(obj_state->getVelocity());
    }
}

void SimEnvWorldStateSpace::extractState(sim_env::WorldConstPtr world, StateType* state) const {
    for (unsigned int i = 0; i < getNumObjects(); ++i) {
        sim_env::ObjectConstPtr object = world->getObjectConst(_object_names[i], false);
        SimEnvObjectState* obj_state = state->getObjectState(i);
        obj_state->setConfiguration(object->getDOFPositions());
        obj_state->setVelocity(object->getDOFVelocities());
    }
}

void SimEnvWorldStateSpace::setDistanceMeasure(StateDistanceMeasureConstPtr measure) {
    _distance_measure = measure;
}

StateDistanceMeasureConstPtr SimEnvWorldStateSpace::getDistanceMeasure() {
    return _distance_measure;
}

void SimEnvWorldStateSpace::constructLimits(sim_env::ObjectConstPtr object, const PlanningSceneBounds& bounds,
                     Eigen::ArrayX2f& position_limits, Eigen::ArrayX2f& velocity_limits) const {
    Eigen::VectorXi active_dofs = object->getActiveDOFs();
    position_limits = object->getDOFPositionLimits();
    velocity_limits = object->getDOFVelocityLimits();
    // if we have a static object, there is nothing more to do
    if (object->isStatic()) {
        return;
    }
    // else we might need to change the limits of dofs related to the object's pose
    // run over potential pose dofs
    for (unsigned int i = 0; i < std::min((unsigned int)active_dofs.size(), object->getNumBaseDOFs()); ++i) {
        if (active_dofs[i] < object->getNumBaseDOFs()) {
            if (object->getNumBaseDOFs() == 3) {
                // we are in a 2d world
                if (active_dofs[i] == 0) {
                    // x
                    position_limits(i, 0) = bounds.x_limits[0];
                    position_limits(i, 1) = bounds.x_limits[1];
                    velocity_limits(i, 0) = -bounds.max_velocity;
                    velocity_limits(i, 1) = bounds.max_velocity;
                } else if (active_dofs[i] == 1) {
                    // y
                    position_limits(i, 0) = bounds.y_limits[0];
                    position_limits(i, 1) = bounds.y_limits[1];
                    velocity_limits(i, 0) = -bounds.max_velocity;
                    velocity_limits(i, 1) = bounds.max_velocity;
                } else {
                    // theta
                    velocity_limits(i, 0) = -bounds.max_rotation_vel;
                    velocity_limits(i, 1) = bounds.max_rotation_vel;
                }
            } else {
                // we are in a 3d world
                assert(object->getNumBaseDOFs() == 6);
                if (active_dofs[i] == 0) {
                    // x
                    position_limits(i, 0) = bounds.x_limits[0];
                    position_limits(i, 1) = bounds.x_limits[1];
                    velocity_limits(i, 0) = -bounds.max_velocity;
                    velocity_limits(i, 1) = bounds.max_velocity;
                } else if (active_dofs[i] == 1) {
                    // y
                    position_limits(i, 0) = bounds.y_limits[0];
                    position_limits(i, 1) = bounds.y_limits[1];
                    velocity_limits(i, 0) = -bounds.max_velocity;
                    velocity_limits(i, 1) = bounds.max_velocity;
                } else if (active_dofs[i] == 2){
                    // z
                    position_limits(i, 0) = bounds.z_limits[0];
                    position_limits(i, 1) = bounds.z_limits[1];
                    velocity_limits(i, 0) = -bounds.max_velocity;
                    velocity_limits(i, 1) = bounds.max_velocity;
                } else {
                    // theta
                    velocity_limits(i, 0) = -bounds.max_rotation_vel;
                    velocity_limits(i, 1) = bounds.max_rotation_vel;
                }
            }
        }
    }
}

////////////////////////////////////////////////////////////////////////////////
////////////////////////////// SimEnvValidityChecker ///////////////////////////
////////////////////////////////////////////////////////////////////////////////
SimEnvValidityChecker::CollisionPolicy::CollisionPolicy() :
        _b_static_col_allowed(true)
{
}

SimEnvValidityChecker::CollisionPolicy::CollisionPolicy(const CollisionPolicy& other) = default;

SimEnvValidityChecker::CollisionPolicy::~CollisionPolicy() = default;

SimEnvValidityChecker::CollisionPolicy& SimEnvValidityChecker::CollisionPolicy::operator=(const CollisionPolicy &other) {
    if (this != &other) {
        _b_static_col_allowed = other._b_static_col_allowed;
        _forbidden_collisions = other._forbidden_collisions;
        _static_collisions = other._static_collisions;
    }
    return *this;
}

void SimEnvValidityChecker::CollisionPolicy::setStaticCollisions(bool allowed) {
    _b_static_col_allowed = allowed;
}

void SimEnvValidityChecker::CollisionPolicy::setStaticCollisions(const std::string& obj, bool allowed) {
    _static_collisions[obj] = allowed;
}

void SimEnvValidityChecker::CollisionPolicy::setStaticCollisions(sim_env::ObjectConstPtr obj, bool allowed) {
    setStaticCollisions(obj->getName(), allowed);
}

bool SimEnvValidityChecker::CollisionPolicy::staticCollisionsAllowed() const {
    return _b_static_col_allowed;
}

bool SimEnvValidityChecker::CollisionPolicy::staticCollisionsAllowed(const std::string& obj) const {
    auto iter = _static_collisions.find(obj);
    if (iter != _static_collisions.end()) {
        return iter->second;
    }
    return staticCollisionsAllowed();
}

bool SimEnvValidityChecker::CollisionPolicy::staticCollisionsAllowed(sim_env::ObjectConstPtr obj) const {
    return staticCollisionsAllowed(obj->getName());
}

void SimEnvValidityChecker::CollisionPolicy::setCollision(const std::string& obj1,
                                                          const std::string& obj2,
                                                          bool allowed) {
    auto pair = std::make_pair(obj1, obj2);
    auto iter = _forbidden_collisions.find(pair);
    if (iter != _forbidden_collisions.end()) { // if we have (obj1, obj2) already stored
        iter->second = allowed;
    } else {
        _forbidden_collisions[pair] = allowed; // else store it
    }
}

void SimEnvValidityChecker::CollisionPolicy::setCollision(sim_env::ObjectConstPtr obj1,
                                                          sim_env::ObjectConstPtr obj2,
                                                          bool allowed) {
    setCollision(obj1->getName(), obj2->getName(), allowed);
}

bool SimEnvValidityChecker::CollisionPolicy::collisionAllowed(const std::string& obj1, const std::string& obj2) const {
    auto pair = std::make_pair(obj1, obj2);
    auto iter = _forbidden_collisions.find(pair);
    if (iter != _forbidden_collisions.end()) {
        return iter->second;
    }
    return true; // if we don't know about it, it is allowed
}

bool SimEnvValidityChecker::CollisionPolicy::collisionAllowed(sim_env::ObjectConstPtr obj1, sim_env::ObjectConstPtr obj2) const {
    if (obj1->isStatic() and obj2->isStatic()) {
        mps_logging::logWarn("Request made whether two static objects are allowed to contact each other.",
                             "[mps::planner::ompl::state::SimEnvValidityChecker::CollisionPolicy::collisionAllowed]");
    }
    if (obj1->isStatic()) {
        return staticCollisionsAllowed(obj2->getName());
    }
    if (obj2->isStatic()) {
        return staticCollisionsAllowed(obj1->getName());
    }
    return collisionAllowed(obj1->getName(), obj2->getName());
}

std::size_t SimEnvValidityChecker::CollisionPolicy::PairHash::operator()(
        const std::pair<std::string, std::string> &key) const {
    auto hash_fn = std::hash<std::string>();
    return key.first == key.second ? hash_fn(key.first) : hash_fn(key.first) ^ hash_fn(key.second);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////// SimEnvValidityChecker /////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////
SimEnvValidityChecker::SimEnvValidityChecker(::ompl::base::SpaceInformationPtr si,
                                             sim_env::WorldPtr world) :
        StateValidityChecker(si), _world(world)
{
    _world_space = std::dynamic_pointer_cast<const SimEnvWorldStateSpace>(si->getStateSpace());
    _world_state = dynamic_cast<mps::planner::ompl::state::SimEnvWorldState*>(_world_space->allocState());
    specs_.clearanceComputationType = ::ompl::base::StateValidityCheckerSpecs::ClearanceComputationType::NONE;
    assert(_world);
    assert(_world_space);
}

SimEnvValidityChecker::~SimEnvValidityChecker() {
    _world_space->freeState(_world_state);
}

bool SimEnvValidityChecker::isValid(const ::ompl::base::State *state) const {
    auto* world_state = state->as<SimEnvWorldState>();
    // first check whether the provided state is within bounds
    bool bounds_valid = _world_space->satisfiesBounds(state);
    if (!bounds_valid) {
        _world->getLogger()->logDebug("State bounds violated. Rejecting state.",
                                      "[mps::planner::ompl::state::SimEnvValidityChecker::isValid]");
        return false;
    }
    // next check, whether the state is valid in terms of collisions
    std::lock_guard<std::recursive_mutex> world_lock(_world->getMutex());
    // save the state the world is in
    _world->saveState();
    // now set it to represent the given SimEnvWorldState
    _world_space->setToState(_world, world_state);
    // first check whether this state is physically feasible
    if (not _world->isPhysicallyFeasible()) {
        _world->getLogger()->logDebug("Rejecting state because it is physically infeasible.",
                                      "[mps::planner::ompl::state::SimEnvValidityChecker::isValid]");
    }
    // check for collisions
    std::vector<sim_env::ObjectPtr> objects;
    _world->getObjects(objects, false);
    for (auto object : objects) {
        std::vector<sim_env::Contact> contacts;
        _world->checkCollision(object, contacts);
        for (auto contact : contacts) {
            bool contact_ok = checkContact(contact);
            if (!contact_ok) {
                _world->getLogger()->logDebug("Rejecting state due to violation of contact constraints.",
                                              "[mps::planner::ompl::state::SimEnvValidityChecker::isValid]");
                return false;
            }
        }
    }
    // finally restore whatever state the world was in before
    _world->restoreState();
    return true;
}

bool SimEnvValidityChecker::isValidIntermediate(const ::ompl::base::State* state) const {
    auto* world_state = state->as<SimEnvWorldState>();
    std::lock_guard<std::recursive_mutex> world_lock(_world->getMutex());
    _world->saveState();
    _world_space->setToState(_world, world_state);
    bool valid = isValidIntermediate();
    _world->restoreState();
    return valid;
}

bool SimEnvValidityChecker::isValidIntermediate() const {
    _world_space->extractState(_world, _world_state);
    // first check whether the current state is within bounds
    bool bounds_valid = _world_space->satisfiesBounds(_world_state);
    if (!bounds_valid) {
        _world->getLogger()->logDebug("State bounds violated. Rejecting state.",
                                      "[mps::planner::ompl::state::SimEnvValidityChecker::isValidIntermediate]");
        return false;
    }
    // next check, whether the state is valid in terms of collisions
    std::lock_guard<std::recursive_mutex> world_lock(_world->getMutex());
    // check for collisions
    std::vector<sim_env::ObjectPtr> objects;
    _world->getObjects(objects, false);
    for (const auto &object : objects) {
        std::vector<sim_env::Contact> contacts;
        _world->checkCollision(object, contacts);
        for (const auto &contact : contacts) {
            bool contact_ok = checkContact(contact);
            if (!contact_ok) {
                _world->getLogger()->logDebug("Rejecting state due to violation of contact constraints.",
                                              "[mps::planner::ompl::state::SimEnvValidityChecker::isValidIntermediate]");
                return false;
            }
        }
    }
    return true;
}

bool SimEnvValidityChecker::checkContact(const sim_env::Contact& contact) const {
    sim_env::ObjectPtr obj1 = contact.object_a.lock();
    sim_env::ObjectPtr obj2 = contact.object_b.lock();
    if (!obj1 or !obj2) {
        throw std::logic_error("[mps::planner::ompl::state::SimEnvValidityChecker::checkContact]"
                                       "Could not lock weak pointer to objects stored in contact.");
    }
    return collision_policy.collisionAllowed(obj1, obj2);
}


