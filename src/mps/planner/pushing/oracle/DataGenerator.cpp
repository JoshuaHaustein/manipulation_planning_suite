//
// Created by joshua on 9/29/17.
//
#include <mps/planner/pushing/oracle/DataGenerator.h>
#include <mps/planner/util/Random.h>
#include <ompl/base/samplers/UniformValidStateSampler.h>

using namespace mps::planner::pushing::oracle;


DataGenerator::DataGenerator(::ompl::control::SpaceInformationPtr si,
                             mps::planner::ompl::control::SimEnvStatePropagatorPtr state_prop,
                             sim_env::WorldPtr world,
                             const std::string& robot_name,
                             const std::string& obj_name) :
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
    _max_distance = 1.0f;  // TODO compute using control limits
    _state_sigma.resize(_object->getNumActiveDOFs());
    _state_sigma.setZero(); // TODO set to reasonable values
}

DataGenerator::~DataGenerator() = default;

void DataGenerator::generateData(const std::string& file_name,
                                 unsigned int num_samples,
                                 const std::string& header,
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
    data_dumper.writeHeader(header);
    if (deterministic) {
        num_noise_samples = 1;
    }
    unsigned int i = 0;
    while (i < num_samples) {
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
//            double state_distance = _space_information->distance(noisy_state, new_state);
//            if (state_distance <= 0.0000001) {
//                _planning_problem.world->getLogger()->logWarn("Resulting state is equal to start state");
//            }
            data_dumper.saveData(mean_state, new_state, control);
        }
        ++i;
    }
    data_dumper.closeFile();
    _space_information->freeState(mean_state);
    _space_information->freeState(new_state);
    _space_information->freeState(noisy_state);
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
    // TODO perturb object data (mass and friction coeffs)
}

void DataGenerator::restoreDynamics() {
    // TODO restore object data
}

