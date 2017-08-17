//
// Created by joshua on 8/16/17.
//
#include <sim_env/utils/EigenUtils.h>
#include <mps/planner/ompl/state/SimEnvState.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SE3StateSpace.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>

using namespace mps::planner::ompl::state;

////////////////////////////////////////////////////////////////////////////////////////
/////////////////// SimEnvObjectVelocity aka SimEnvObjectVelocitySpace::StateType //////
////////////////////////////////////////////////////////////////////////////////////////
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
                ::ompl::base::RealVectorStateSpace::StateType* real_vector_state =
                        static_cast<::ompl::base::RealVectorStateSpace::StateType*>(components[space_idx]);
                for (unsigned int i = 0; i < _state_descriptions.at(space_idx).dim; ++i) {
                    config[dof_idx] = (float) real_vector_state->values[i];
                    dof_idx++;
                }
                break;
            }
            case internal::SubspaceType::SO3:
            {
                ::ompl::base::SO3StateSpace::StateType* so3_state =
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
                ::ompl::base::SO2StateSpace::StateType* so2_state =
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
                ::ompl::base::RealVectorStateSpace::StateType* real_vector_state =
                        static_cast<::ompl::base::RealVectorStateSpace::StateType*>(components[space_idx]);
                for (unsigned int i = 0; i < _state_descriptions.at(space_idx).dim; ++i) {
                    real_vector_state->values[i] = (double) config[dof_idx++];
                }
                break;
            }
            case internal::SubspaceType::SO3:
            {
                ::ompl::base::SO3StateSpace::StateType* so3_state =
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
                ::ompl::base::SO2StateSpace::StateType* so2_state =
                        static_cast<::ompl::base::SO2StateSpace::StateType*>(components[space_idx]);
                so2_state->value =(double)config[dof_idx++];
                break;
            }
        }
    }
}

////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////// SimEnvObjectConfigurationSpace ////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
SimEnvObjectConfigurationSpace::SimEnvObjectConfigurationSpace(sim_env::ObjectConstPtr object,
                                                               double pos_weight,
                                                               double orientation_weight,
                                                               double joint_weight) :
    _object(object),
    _active_dofs(object->getActiveDOFs()),
    _position_weight(pos_weight),
    _orientation_weight(orientation_weight),
    _joint_weight(joint_weight)
{
    assert(_active_dofs.size() > 0);
    setName("sim_env::Object(" + object->getName() + ")-ConfigurationSpace");
    // TODO what about bounds?
    addPoseSubspace(object);
    addJointsSubspace(object);
    lock();
}

SimEnvObjectConfigurationSpace::~SimEnvObjectConfigurationSpace() {
}

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

void SimEnvObjectConfigurationSpace::addPoseSubspace(sim_env::ObjectConstPtr object) {
    if (object->isStatic()) { // no pose subspace to add
        return;
    }
    // let's first create a helper vector for which idx_helper[i] = _active_dofs[i] if i < _active_dofs.size(),
    // else idx_helper[i] = -1 for i = 0 .. 5
    Eigen::VectorXi idx_helper(6);
    for (unsigned int i = 0; i < idx_helper.size(); ++i) {
        idx_helper[i] = _active_dofs.size() > i + 1 ? _active_dofs[i] : -1;
    }
    // if the object is not static we need to distinguish two different cases
    // either we are in a 2d world (getNumBaseDOFs == 3) or in a 3d world (getNumBaseDOFs == 6)
    if (object->getNumBaseDOFs() == 3) { // we are in a 2d world
        if (idx_helper[0] == 0 and idx_helper[1] == 1) {
            // we have both x and y, so let's add a 2d real vector space
            ::ompl::base::StateSpacePtr pos_state_space = std::make_shared<::ompl::base::RealVectorStateSpace>(2);
            pos_state_space->setName(internal::createSpaceName({0,1}, "ConfigurationSpace"));
            addSubspace(pos_state_space, _position_weight);
            internal::SubstateTypeDescription desc(internal::SubspaceType::RealVector, 2);
            _state_descriptions.push_back(desc);
            addSingleBasePoseSpace2D(idx_helper[2]); // and maybe also SO
        } else {
            // else we can treat each potential base dof independently
            addSingleBasePoseSpace2D(idx_helper[0]); // might be position or orientation
            addSingleBasePoseSpace2D(idx_helper[1]); // might be orientation
        }
    } else {
        assert(object->getNumBaseDOFs() == 6); // we are in a 3d world
        unsigned int rdio = 0; // rotation dof index offset
        if (idx_helper[0] == 0 and idx_helper[1] == 1 and idx_helper[2] == 2) {
            // all position dimensions are part of the configspace
            ::ompl::base::StateSpacePtr pos_state_space = std::make_shared<::ompl::base::RealVectorStateSpace>(3);
            pos_state_space->setName(internal::createSpaceName({0,1,2}, "ConfigurationSpace"));
            addSubspace(pos_state_space, _position_weight);
            internal::SubstateTypeDescription desc(internal::SubspaceType::RealVector, 3);
            _state_descriptions.push_back(desc);
            rdio = 3;
        } else if ((idx_helper[0] == 0 and idx_helper[1] == 1) or
                (idx_helper[0] == 0 and idx_helper[1] == 2) or
                (idx_helper[0] == 1 and idx_helper[1] == 2)) {
            // only 2 position dimensions are part of the configspace
            ::ompl::base::StateSpacePtr pos_state_space = std::make_shared<::ompl::base::RealVectorStateSpace>(2);
            pos_state_space->setName(internal::createSpaceName({idx_helper[0], idx_helper[1]}, "ConfigurationSpace"));
            addSubspace(pos_state_space, _position_weight);
            internal::SubstateTypeDescription desc(internal::SubspaceType::RealVector, 2);
            _state_descriptions.push_back(desc);
            rdio = 2;
        } else {
            // at most one dimension is part of the config space (active_dofs should be sorted)
            addSingleBasePoseSpace3D(idx_helper[0]);
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
            addSingleBasePoseSpace3D(idx_helper[rdio]);
            addSingleBasePoseSpace3D(idx_helper[rdio + 1]);
        }
    }
}

void SimEnvObjectConfigurationSpace::addJointsSubspace(sim_env::ObjectConstPtr object) {
    std::vector<int> joint_idx;
    for (unsigned int i = 0; i < _active_dofs.size(); ++i) {
        int dof = _active_dofs[i];
        if (dof > object->getNumBaseDOFs()) {
            joint_idx.push_back(dof);
        }
    }
    if (joint_idx.size() == 0) return;
    ::ompl::base::StateSpacePtr space = std::make_shared<::ompl::base::RealVectorStateSpace>(joint_idx.size());
    space->setName(internal::createSpaceName(joint_idx, "ConfigurationSpace"));
    addSubspace(space, _joint_weight);
    _state_descriptions.push_back(internal::SubstateTypeDescription(internal::SubspaceType::RealVector,
                                                                    (unsigned int) joint_idx.size()));
}

void SimEnvObjectConfigurationSpace::addSingleBasePoseSpace2D(int dof_idx) {
    ::ompl::base::StateSpacePtr space;
    internal::SubstateTypeDescription type_desc;
    type_desc.dim = 1;
    double weight = 1.0;
    if (dof_idx == 0 or dof_idx == 1) { // check whether its a base position
        space = std::make_shared<::ompl::base::RealVectorStateSpace>(1);
        type_desc.type = internal::SubspaceType::RealVector;
        weight = _position_weight;
    } else if (dof_idx == 2) { // or base orientation
        space = std::make_shared<::ompl::base::SO2StateSpace>();
        weight = _orientation_weight;
        type_desc.type = internal::SubspaceType::SO2;
    } else {
        return; // or some other odf, then we don't need to add it
    }
    space->setName(internal::createSpaceName({dof_idx}, "ConfigurationSpace"));
    addSubspace(space, weight);
    _state_descriptions.push_back(type_desc);
}

void SimEnvObjectConfigurationSpace::addSingleBasePoseSpace3D(int dof_idx) {
    ::ompl::base::StateSpacePtr space;
    double weight = 1.0;
    internal::SubstateTypeDescription type_desc;
    type_desc.dim = 1;
    if (dof_idx == 0 or dof_idx == 1 or dof_idx == 2) { // check whether it's a base position
        space = std::make_shared<::ompl::base::RealVectorStateSpace>(1);
        weight = _position_weight;
        type_desc.type = internal::SubspaceType::RealVector;
    } else if (dof_idx == 3 or dof_idx == 4 or dof_idx == 5) { // else it's a base orientation
        space = std::make_shared<::ompl::base::SO2StateSpace>();
        weight = _orientation_weight;
        type_desc.type = internal::SubspaceType::SO2;
    } else {
        return; // do not proceed further
    }
    space->setName(internal::createSpaceName({dof_idx}, "ConfigurationSpace"));
    addSubspace(space, weight);
    _state_descriptions.push_back(type_desc);
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

////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////// SimEnvObjectVelocitySpace ////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
SimEnvObjectVelocitySpace::SimEnvObjectVelocitySpace(sim_env::ObjectConstPtr object,
                                                     double pos_weight,
                                                     double orientation_weight,
                                                     double joint_weight) :
    _object(object),
    _active_dofs(object->getActiveDOFs()),
    _position_vel_weight(pos_weight),
    _orientation_vel_weight(orientation_weight),
    _joint_vel_weight(joint_weight)
{
    assert(_active_dofs.size() > 0);
    setName("sim_env::Object(" + object->getName() + ")-VelocitySpace");
    // TODO what about bounds?
    addPoseVelocitySubspace(object);
    addJointsVelocitySubspace(object);
    lock();
}

SimEnvObjectVelocitySpace::~SimEnvObjectVelocitySpace() {
}

::ompl::base::State* SimEnvObjectVelocitySpace::allocState() const {
    SimEnvObjectVelocity* velocity = new SimEnvObjectVelocity(_state_descriptions, _active_dofs.size());
    allocStateComponents(velocity);
    return velocity;
}

void SimEnvObjectVelocitySpace::freeState(::ompl::base::State* state) const {
    SimEnvObjectVelocity* velocity = static_cast<SimEnvObjectVelocity*>(state);
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
    std::vector<int> orientation_dofs;
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
        }
        if (max_num_position_dofs < _active_dofs[i] and
            _active_dofs[i] < max_num_position_dofs + max_num_orientation_dofs) {
            orientation_dofs.push_back(_active_dofs[i]);
        }
    }
    if (position_dofs.size() > 0) {
        ::ompl::base::StateSpacePtr velocity_space = std::make_shared<::ompl::base::RealVectorStateSpace>(position_dofs.size());
        velocity_space->setName(internal::createSpaceName(position_dofs, "VelocitySpace"));
        addSubspace(velocity_space, _position_vel_weight);
        _state_descriptions.push_back(internal::SubstateTypeDescription(internal::SubspaceType::RealVector,
                                                                        (unsigned int) position_dofs.size()));
    }
    if (orientation_dofs.size() > 0) {
        ::ompl::base::StateSpacePtr velocity_space = std::make_shared<::ompl::base::RealVectorStateSpace>(orientation_dofs.size());
        velocity_space->setName(internal::createSpaceName(orientation_dofs, "VelocitySpace"));
        addSubspace(velocity_space, _orientation_vel_weight);
        _state_descriptions.push_back(internal::SubstateTypeDescription(internal::SubspaceType::RealVector,
                                                                        (unsigned int) orientation_dofs.size()));
    }
}

void SimEnvObjectVelocitySpace::addJointsVelocitySubspace(sim_env::ObjectConstPtr object) {
    std::vector<int> joint_idx;
    for (unsigned int i = 0; i < _active_dofs.size(); ++i) {
        int dof = _active_dofs[i];
        if (dof > object->getNumBaseDOFs()) {
            joint_idx.push_back(dof);
        }
    }
    if (joint_idx.size() == 0) return;
    ::ompl::base::StateSpacePtr space = std::make_shared<::ompl::base::RealVectorStateSpace>(joint_idx.size());
    space->setName(internal::createSpaceName(joint_idx, "VelocitySpace"));
    addSubspace(space, _joint_vel_weight);
    _state_descriptions.push_back(internal::SubstateTypeDescription(internal::SubspaceType::RealVector,
                                                                    (unsigned int) joint_idx.size()));
}

////////////////////////////////////////////////////////////////////////////////////////
/////////////////// SimEnvObjectState aka SimEnvObjectState::StateType /////////////////
////////////////////////////////////////////////////////////////////////////////////////
SimEnvObjectStateSpace::StateType::StateType(bool configuration_only) :
        _configuration_only(configuration_only)
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
        return;
    }
    auto* vel_component = components[1]->as<SimEnvObjectVelocity>();
    vel_component->getVelocity(vel);
}

void SimEnvObjectStateSpace::StateType::setVelocity(const Eigen::VectorXf& vel) {
    auto* vel_component = components[1]->as<SimEnvObjectVelocity>();
    vel_component->setVelocity(vel);
}

////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////// SimEnvObjectStateSpace ////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////
SimEnvObjectStateSpace::SimEnvObjectStateSpace(sim_env::ObjectConstPtr object,
                                               const DistanceWeights& weights,
                                               bool position_only):
        _object(object),
        _active_dofs(object->getActiveDOFs()),
        _position_only(position_only)
{
    assert(_active_dofs.size() > 0);
    setName("sim_env::Object(" + object->getName() + ")-StateSpace");
    // TODO what about bounds?
    ::ompl::base::StateSpacePtr configuration_space =
            std::make_shared<SimEnvObjectConfigurationSpace>(object,
                                                             weights.position_weight,
                                                             weights.orientation_weight,
                                                             weights.joint_weight);
    addSubspace(configuration_space, weights.configuration_weight);
    if (not position_only) {
        ::ompl::base::StateSpacePtr velocity_space =
                std::make_shared<SimEnvObjectVelocitySpace>(object,
                                                            weights.position_weight,
                                                            weights.orientation_weight,
                                                            weights.joint_weight);
        addSubspace(velocity_space, weights.velocity_weight);
    }
    lock();
}

SimEnvObjectStateSpace::~SimEnvObjectStateSpace() {
}

::ompl::base::State* SimEnvObjectStateSpace::allocState() const {
    SimEnvObjectState* object_state = new SimEnvObjectState(_position_only);
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

////////////////////////////////////////////////////////////////////////////////
////////////////////////////// SimEnvWorldStateSpace ///////////////////////////
////////////////////////////////////////////////////////////////////////////////
SimEnvWorldStateSpace::SimEnvWorldStateSpace(sim_env::WorldConstPtr world):
    _world(world)
{
    std::vector<sim_env::ObjectConstPtr> objects;
    world->getObjects(objects, false);
    for (auto& object : objects) {
        if (object->getNumActiveDOFs() > 0) {
            SimEnvObjectStateSpacePtr state_space = std::make_shared<SimEnvObjectStateSpace>(object);
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

sim_env::ObjectConstPtr SimEnvWorldStateSpace::getObject(unsigned int i) const {
    if (i > _object_names.size()) {
        return nullptr;
    }
    sim_env::WorldConstPtr world = _world.lock();
    if (!world) {
        throw std::logic_error("[mps::planner::ompl::state::SimEnvWorldStateSpace::getObject]"
            " The world of this state space does no longer exist.");
    }
    return world->getObjectConst(_object_names[i], false);
}

unsigned int SimEnvWorldStateSpace::getNumObjects() const {
    return (unsigned int) _object_names.size();
}
