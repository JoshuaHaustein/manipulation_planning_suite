//
// Created by joshua on 9/29/17.
//
#include <iostream>
#include <mps/planner/pushing/oracle/DataGenerator.h>
#include <mps/planner/util/Random.h>
#include <mps/planner/pushing/OraclePushPlanner.h>
#include <mps/planner/util/yaml/OracleParsing.h>
#include <ompl/base/samplers/UniformValidStateSampler.h>
#include <boost/math/constants/constants.hpp>

using namespace mps::planner::pushing::oracle;

DataGenerator::Parameters::Parameters() {
    obj_orientation_sigma = boost::math::float_constants::pi / 18.0f; // 10 degrees
    obj_position_sigma = 0.01; // 1 cm
    mass_sigma = 0.002;
    friction_sigma = 0.004;
}

DataGenerator::DataGenerator(::ompl::control::SpaceInformationPtr si,
                             mps::planner::ompl::control::SimEnvStatePropagatorPtr state_prop,
                             sim_env::WorldPtr world,
                             const std::string& robot_name,
                             const std::string& obj_name,
                             const Parameters& params) :
        _world(world),
        _space_information(si),
        _state_propagator(state_prop)
{
    _robot = world->getRobot(robot_name);
    _object = world->getObject(obj_name);
    auto state_space = std::dynamic_pointer_cast<ompl::state::SimEnvWorldStateSpace>(si->getStateSpace());
    int robot_id = state_space->getObjectIndex(_robot->getName());
    assert(robot_id >= 0);
    _robot_id = (unsigned int)robot_id;
    int object_id = state_space->getObjectIndex(_object->getName());
    assert(object_id >= 0);
    _object_id = (unsigned int)object_id;
    assert(_robot);
    assert(_object);
    computeMaxDistance();
    _state_sigma.resize(_object->getNumActiveDOFs());
    _state_sigma.setZero();
    assert(_state_sigma.size() >= 3);
    _state_sigma[0] = params.obj_position_sigma;
    _state_sigma[1] = params.obj_position_sigma;
    _state_sigma[2] = params.obj_orientation_sigma;
    _mass_stddev = params.mass_sigma;
    _friction_stddev = params.friction_sigma;
}

DataGenerator::~DataGenerator() = default;

void DataGenerator::generateData(const std::string& file_name,
                                 unsigned int num_samples,
                                 const std::string& annotation,
                                 bool deterministic,
                                 unsigned int num_noise_samples)
{
    ::ompl::control::ControlSamplerPtr control_sampler = _space_information->allocControlSampler();
    auto state_sampler = _space_information->getStateSpace()->allocStateSampler();
    auto validity_checker = _space_information->getStateValidityChecker();
    ::ompl::base::State* mean_state = _space_information->allocState();
    ::ompl::base::State* noisy_state = _space_information->allocState();
    ::ompl::base::State* new_state = _space_information->allocState();
    ::ompl::control::Control* control = _space_information->allocControl();
    mps::planner::util::serialize::OracleDataDumper data_dumper;
    data_dumper.setFile(file_name);
    data_dumper.openFile();
    if (deterministic) {
        num_noise_samples = 1;
    }
    unsigned int i = 0;
    while (i < num_samples) {
//        _world->getLogger()->logInfo(boost::format("Sampling sample %i") % i, "[DataGenerator]");
        bool has_state = sampleValidState(mean_state, state_sampler, validity_checker);
        if (not has_state) {
            _world->getLogger()->logWarn("Failed to sample a valid state, we might end up in a infinite loop here!");
            continue;
        }
        control_sampler->sample(control);
        _world->getLogger()->logDebug("Sampled state");
        _space_information->copyState(noisy_state, mean_state);
        for (unsigned int j = 0; j < num_noise_samples; ++j) {
            if (not deterministic) {
                applyNoise(mean_state, noisy_state);
                modifyDynamics();
            }
            bool success = _state_propagator->propagate(noisy_state, control, new_state);
            if (not deterministic) {
                restoreDynamics();
            }
            _world->getLogger()->logDebug("Propagated state");
            if (not success) {
                _world->getLogger()->logDebug("State propagation failed, skipping");
                continue;
            }
            data_dumper.saveData(noisy_state, new_state, control, annotation);
        }
        ++i;
    }
    data_dumper.closeFile();
    _space_information->freeState(mean_state);
    _space_information->freeState(new_state);
    _space_information->freeState(noisy_state);
    _space_information->freeControl(control);
}

void DataGenerator::evaluateOracle(mps::planner::ompl::state::goal::RelocationGoalSpecification goal,
                                   mps::planner::pushing::oracle::OracleControlSamplerPtr oracle_sampler,
                                   mps::planner::ompl::state::SimEnvWorldStateSpacePtr state_space,
                                   const std::string& file_name,
                                   unsigned int num_samples,
                                   const std::string& annotation)
{
    ::ompl::control::ControlSamplerPtr control_sampler = _space_information->allocControlSampler();
    auto state_sampler = _space_information->getStateSpace()->allocStateSampler();
    auto validity_checker = _space_information->getStateValidityChecker();
    ::ompl::base::State* start_state = _space_information->allocState();
    ::ompl::base::State* feasible_state = _space_information->allocState();
    ::ompl::base::State* goal_state = _space_information->allocState();
    ::ompl::base::State* final_state = _space_information->allocState();
    ::ompl::control::Control* control = _space_information->allocControl();
    mps::planner::util::serialize::OracleDataDumper data_dumper;
    data_dumper.setFile(file_name);
    data_dumper.openFile();
    unsigned int i = 0;
    while (i < num_samples) {
        bool has_states;
        has_states = sampleValidState(start_state, state_sampler, validity_checker);
        if (not has_states) {
            _world->getLogger()->logWarn("Failed to sample a valid state, we might end up in a infinite loop here!");
            continue;
        }
        has_states = sampleValidState(goal_state, state_sampler, validity_checker);
        if (not has_states) {
            _world->getLogger()->logWarn("Failed to sample a valid state, we might end up in a infinite loop here!");
            continue;
        }

        std::cout << start_state << goal_state << std::endl;

        std::vector<::ompl::control::Control const*> oracle_controls;
        bool success = false;

        int target_id = state_space->getObjectIndex(goal.object_name);

        success = oracle_sampler->steerRobot(oracle_controls, start_state, goal_state);
        if (not success) {
            _world->getLogger()->logWarn("State propagation to feasible state failed, skipping");
            continue;
        }

        // TODO change to steer from achieved feasible state
        success = oracle_sampler->steerPush(oracle_controls, start_state, goal_state, target_id);
        if (not success) {
            _world->getLogger()->logWarn("State propagation for pushing failed, skipping");
            continue;
        }

        control_sampler->sample(control);
        _world->getLogger()->logDebug("Sampled state");
        success = _state_propagator->propagate(start_state, control, final_state);
        _world->getLogger()->logDebug("Propagated state");
        if (not success) {
            _world->getLogger()->logWarn("State propagation to goal state failed, skipping");
            continue;
        }
        data_dumper.saveData(goal_state, final_state, control, annotation);
        std::cout << "Finished one trial" << std::endl;
        ++i;
    }
    _space_information->freeState(start_state);
    _space_information->freeState(feasible_state);
    _space_information->freeState(goal_state);
    _space_information->freeState(final_state);
    _space_information->freeControl(control);
}

bool DataGenerator::sampleValidState(::ompl::base::State *state,
                                     ::ompl::base::StateSamplerPtr state_sampler,
                                     ::ompl::base::StateValidityCheckerPtr validity_checker)
{
    unsigned int num_tries = 100;
    bool valid_sample = false;
    for (unsigned int i = 0; i < num_tries; ++i) {
        state_sampler->sampleUniform(state);
        auto sim_env_state = dynamic_cast<ompl::state::SimEnvWorldState*>(state);
        assert(sim_env_state);
        auto robot_state = sim_env_state->getObjectState(_robot_id);
        Eigen::VectorXf config;
        robot_state->getConfiguration(config);
        config.setZero();
        robot_state->setConfiguration(config);
        // check for validity
        valid_sample = validity_checker->isValid(state);
        auto object_state = sim_env_state->getObjectState(_object_id);
        object_state->getConfiguration(config);
        float distance = config.head(2).norm();
        valid_sample = valid_sample and distance < _max_distance;
        if (valid_sample) break;
    }
    return valid_sample;
}

void DataGenerator::applyNoise(const ::ompl::base::State *mean_state, ::ompl::base::State *noisy_state) {
    _space_information->copyState(noisy_state, mean_state);
    auto sim_env_state = noisy_state->as<ompl::state::SimEnvWorldState>();
    assert(sim_env_state);
    auto object_state = sim_env_state->getObjectState(_object_id);
    Eigen::VectorXf config;
    object_state->getConfiguration(config);
    auto rng = util::random::getDefaultRandomGenerator();
    for (unsigned int idx = 0; idx < config.size(); ++idx) {
        auto value = rng->gaussian(0.0, _state_sigma[idx]);
        config[idx] += value;
    }
    object_state->setConfiguration(config);
}

void DataGenerator::modifyDynamics() {
    _original_dynamics.mass = _object->getMass();
    _original_dynamics.friction_coeff = _object->getGroundFriction();
    auto rng = util::random::getDefaultRandomGenerator();
    float new_mass = std::max((float)(rng->gaussian(_original_dynamics.mass, _mass_stddev)), 0.000001f);
    float new_friction = std::max((float)rng->gaussian(_original_dynamics.friction_coeff, _friction_stddev), 0.00000001f);
    _object->getBaseLink()->setMass(new_mass);
    _object->getBaseLink()->setGroundFriction(new_friction);
}

void DataGenerator::restoreDynamics() {
    _object->getBaseLink()->setMass(_original_dynamics.mass);
    _object->getBaseLink()->setGroundFriction(_original_dynamics.friction_coeff);
}

void DataGenerator::computeMaxDistance() {
    auto control_space = std::dynamic_pointer_cast<ompl::control::RampVelocityControlSpace>(_space_information->getControlSpace());
    Eigen::Array2f duration_limits;
    Eigen::VectorXf velocity_limits;
    Eigen::VectorXf acceleration_limits;
    control_space->getDurationLimits(duration_limits);
    control_space->getVelocityLimits(velocity_limits);
    control_space->getAccelerationLimits(acceleration_limits);
    float max_acceleration_time = velocity_limits[0] / acceleration_limits[0];
    _max_distance = (max_acceleration_time + duration_limits[1]) * velocity_limits[0];
}

bool DataGenerator::objectMoved(const ::ompl::base::State* initial_state,
                                const ::ompl::base::State* result_state) const {
    auto sim_initial_state = dynamic_cast<const ompl::state::SimEnvWorldState*>(initial_state);
    auto sim_result_state = dynamic_cast<const ompl::state::SimEnvWorldState*>(result_state);
    auto initial_obj_state = sim_initial_state->getObjectState(_object_id);
    auto result_obj_state = sim_result_state->getObjectState(_object_id);
//    std::cout << "Initial state was ";
//    sim_initial_state->print(std::cout);
//    std::cout << " Resulting state was ";
//    sim_result_state->print(std::cout);
//    std::cout << std::endl;
    Eigen::VectorXf initial_config;
    initial_obj_state->getConfiguration(initial_config);
    Eigen::VectorXf result_config;
    result_obj_state->getConfiguration(result_config);
    return (initial_config - result_config).norm() > 0.00001f;
}

