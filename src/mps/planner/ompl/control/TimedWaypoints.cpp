#include <algorithm>
#include <mps/planner/ompl/control/TimedWaypoints.h>

using namespace mps::planner::ompl::control;

// TimedWaypoints::TimedWaypoints(const Eigen::Affine3f& tf) :
// _tf(tf){

// }

TimedWaypoints::TimedWaypoints()
    : _resting_time(0.0f)
{
}

TimedWaypoints::TimedWaypoints(const TimedWaypoints& other)
{
    _waypoints = other._waypoints;
    _resting_time = other._resting_time;
}

TimedWaypoints::TimedWaypoints(
    const std::vector<std::pair<float, Eigen::VectorXf>>& waypoints)
    : _waypoints(waypoints)
{
    if (_waypoints.empty()) {
        throw std::runtime_error("Illegal input arguments. There must be at least one waypoint.");
    }
}

TimedWaypoints::~TimedWaypoints()
{
}

void TimedWaypoints::setWaypoints(const std::vector<std::pair<float, Eigen::VectorXf>>& waypoints)
{
    _waypoints = waypoints;
}

void TimedWaypoints::addWaypoint(float time, const Eigen::VectorXf& wp)
{
    assert(_waypoints.empty() || time > _waypoints.back().first);
    _waypoints.push_back(std::make_pair(time, wp));
}

void TimedWaypoints::reset()
{
    _waypoints.clear();
    _resting_time = 0.0f;
}

bool TimedWaypoints::operator==(const TimedWaypoints& other) const
{
    // TODO how to compare equality between projection functions?
    return _waypoints == other._waypoints and _resting_time == other._resting_time;
}

TimedWaypoints& TimedWaypoints::operator=(const TimedWaypoints& other)
{
    _waypoints = other._waypoints;
    _resting_time = other._resting_time;
    _vel_proj_fn = other._vel_proj_fn;
    return *this;
}

void TimedWaypoints::print(std::ostream& out) const
{
    for (auto& wp : _waypoints) {
        out << "[" << wp.first << ": " << wp.second << "], ";
    }
    out << std::endl;
}

sim_env::RobotController::VelocityProjectionFn TimedWaypoints::getVelocityConstraintProjection() const
{
    return _vel_proj_fn;
}

void TimedWaypoints::setVelocityProjectionFunction(sim_env::RobotController::VelocityProjectionFn fn)
{
    _vel_proj_fn = fn;
}

Eigen::VectorXf TimedWaypoints::getPosition(float t) const
{
    Eigen::VectorXf pos;
    getPosition(t, pos);
    return pos;
}

void TimedWaypoints::getPosition(float t, Eigen::VectorXf& position) const
{
    assert(!_waypoints.empty());
    if (_waypoints.size() == 1) {
        position = _waypoints.at(0).second;
        return;
    }
    // assuming waypoint[0].first == 0.0
    // get first waypoint with wp_t > t
    auto comparer = [](const float& a, const std::pair<float, Eigen::VectorXf>& b) -> bool { return a < b.first; };
    auto second_wp = std::upper_bound(_waypoints.begin(), _waypoints.end(), t, comparer);
    if (second_wp == _waypoints.end()) { // t is past the duration
        position = _waypoints.back().second;
    }
    auto first_wp = second_wp - 1;
    // interpolate linearly between waypoint positions
    float delta_t = second_wp->first - first_wp->first;
    assert(delta_t > 0.0);
    float rel_t = (t - first_wp->first) / delta_t;
    position = rel_t * second_wp->second + (1.0 - rel_t) * first_wp->second;
}

float TimedWaypoints::getDuration() const
{
    return getPreRestDuration() + getRestDuration();
}

float TimedWaypoints::getPreRestDuration() const
{
    if (_waypoints.empty())
        return 0.0f;
    return _waypoints.back().first;
}

float TimedWaypoints::getRestDuration() const
{
    return _resting_time;
}

void TimedWaypoints::addRestDuration(float dur)
{
    _resting_time += dur;
}

void TimedWaypoints::setRestDuration(float dur)
{
    _resting_time = dur;
}

/******************** TimedWaypointsControlSpace *********************/
TimedWaypointsControlSpace::TimedWaypointsControlSpace(const ::ompl::base::StateSpacePtr& state_space)
    : ControlSpace(state_space)
{
}

TimedWaypointsControlSpace::~TimedWaypointsControlSpace() = default;

unsigned int TimedWaypointsControlSpace::getDimension() const
{
    // TODO what to return?
    return stateSpace_->getDimension();
}

::ompl::control::Control* TimedWaypointsControlSpace::allocControl() const
{
    ::ompl::control::Control* new_control = nullptr;
    if (_control_cache.size()) {
        new_control = _control_cache.top();
        _control_cache.pop();
    } else {
        new_control = new TimedWaypoints();
    }
    return new_control;
}

void TimedWaypointsControlSpace::freeControl(::ompl::control::Control* control) const
{
    auto wp_control = control->as<TimedWaypoints>();
    wp_control->reset();
    _control_cache.push(wp_control);
}

void TimedWaypointsControlSpace::copyControl(::ompl::control::Control* control,
    const ::ompl::control::Control* source) const
{
    auto wp_control = control->as<TimedWaypoints>();
    auto wp_source = source->as<TimedWaypoints>();
    wp_control->operator=(*wp_source);
}

bool TimedWaypointsControlSpace::equalControls(const ::ompl::control::Control* control_1,
    const ::ompl::control::Control* control_2) const
{
    auto wp_control_1 = control_1->as<TimedWaypoints>();
    auto wp_control_2 = control_2->as<TimedWaypoints>();
    return wp_control_1->operator==(*wp_control_2);
}

void TimedWaypointsControlSpace::nullControl(::ompl::control::Control* control) const
{
    auto wp_control = control->as<TimedWaypoints>();
    wp_control->reset();
}

void TimedWaypointsControlSpace::printControl(const ::ompl::control::Control* control, std::ostream& out) const
{
    auto wp_control = control->as<TimedWaypoints>();
    wp_control->print(out);
}

void TimedWaypointsControlSpace::setup()
{
    ::ompl::control::ControlSpace::setup();
}

bool TimedWaypointsControlSpace::isCompound() const
{
    return false;
}

void TimedWaypointsControlSpace::serializeSpaceInformation(std::ostream& ostream) const
{
    // TODO anything to store here?
}

bool TimedWaypointsControlSpace::deserializeSpaceInformation(std::istream& istream)
{
    return true;
}

::ompl::control::ControlSamplerPtr TimedWaypointsControlSpace::allocDefaultControlSampler() const
{
    return std::make_shared<TimedWaypointsControlSampler>(shared_from_this());
}

void TimedWaypointsControlSpace::printSettings(std::ostream& out) const
{
    throw std::logic_error("TimedWaypointsControlSpace::allocDefaultControlSampler is not implemented");
}

unsigned int TimedWaypointsControlSpace::getNumParameters() const
{
    throw std::logic_error("TimedWaypointsControlSpace::getNumParameters() is not implemented");
}

unsigned int TimedWaypointsControlSpace::getSerializationLength() const
{
    throw std::logic_error("TimedWaypointsControlSpace::getSerializationLength() is not implemented");
}

void TimedWaypointsControlSpace::serialize(void* serialization, const ::ompl::control::Control* ctrl) const
{
    throw std::logic_error("TimedWaypointsControlSpace::serialize() is not implemented");
}

void TimedWaypointsControlSpace::deserialize(::ompl::control::Control* ctrl, const void* serialization) const
{
    throw std::logic_error("TimedWaypointsControlSpace::deserialize() is not implemented");
}

double* TimedWaypointsControlSpace::getValueAddressAtIndex(::ompl::control::Control* control, unsigned int index) const
{
    throw std::logic_error("TimedWaypointsControlSpace::getValueAddressAtIndex() is not implemented");
}

TimedWaypointsControlSampler::TimedWaypointsControlSampler(const TimedWaypointsControlSpaceConstPtr cspace)
    : ::ompl::control::ControlSampler(cspace.get())
{
}

TimedWaypointsControlSampler::~TimedWaypointsControlSampler()
{
}

void TimedWaypointsControlSampler::sample(::ompl::control::Control* control)
{
    throw std::logic_error("TimedWaypointsControlSampler::sample not implemented yet");
}

TimedWaypointsRobotOracle::TimedWaypointsRobotOracle(ompl::state::SimEnvObjectStateSpacePtr state_space,
    unsigned int robot_id, TimedWaypointsControlSpacePtr control_space, const std::vector<float>& vel_limits)
    : _config_space(state_space->getConfigurationSpace())
    , _control_space(control_space)
    , _robot_id(robot_id)
    , _max_vel(vel_limits)
{
    auto num_subspaces = _config_space->getSubspaceCount();
    assert(_max_vel.size() == num_subspaces);
    // assert(_max_vel.size() == _state_space->getSubspaceCount());
    _dummy_state_a = dynamic_cast<ompl::state::SimEnvObjectConfiguration*>(_config_space->allocState());
    _dummy_state_b = dynamic_cast<ompl::state::SimEnvObjectConfiguration*>(_config_space->allocState());
}

TimedWaypointsRobotOracle::~TimedWaypointsRobotOracle()
{
    _config_space->freeState(_dummy_state_a);
    _config_space->freeState(_dummy_state_b);
}

void TimedWaypointsRobotOracle::steer(const ompl::state::SimEnvObjectState* current_robot_state,
    const ompl::state::SimEnvObjectState* desired_robot_state,
    std::vector<::ompl::control::Control*>& controls) const
{
    current_robot_state->getConfiguration(_eigen_config_a);
    desired_robot_state->getConfiguration(_eigen_config_b);
    steer(_eigen_config_a, _eigen_config_b, controls);
}

void TimedWaypointsRobotOracle::steer(const ompl::state::SimEnvWorldState* current_state,
    const ompl::state::SimEnvObjectState* desired_robot_state,
    std::vector<::ompl::control::Control*>& controls) const
{
    steer(current_state->getObjectState(_robot_id), desired_robot_state, controls);
}

void TimedWaypointsRobotOracle::steer(const Eigen::VectorXf& current_robot_state,
    const Eigen::VectorXf& target_robot_state,
    ::ompl::control::Control* control) const
{
    auto wp_control = dynamic_cast<TimedWaypoints*>(control);
    // add first state
    wp_control->addWaypoint(0.0, current_robot_state);
    // compute distance for each sub-state sapce
    _dummy_state_a->setConfiguration(current_robot_state);
    _dummy_state_b->setConfiguration(target_robot_state);
    float duration = 0.0;
    for (unsigned int i = 0; i < _config_space->getSubspaceCount(); ++i) {
        auto sub_space = _config_space->getSubspace(i);
        float dist = sub_space->distance(_dummy_state_a->components[i], _dummy_state_b->components[i]);
        duration = std::max(dist / _max_vel.at(i), duration);
    }
    // add destination
    wp_control->addWaypoint(duration, target_robot_state);
}

void TimedWaypointsRobotOracle::steer(const Eigen::VectorXf& current_robot_state,
    const Eigen::VectorXf& target_robot_state,
    std::vector<::ompl::control::Control*>& controls) const
{
    auto control = static_cast<TimedWaypoints*>(_control_space->allocControl());
    steer(current_robot_state, target_robot_state, control);
    controls.push_back(control);
}