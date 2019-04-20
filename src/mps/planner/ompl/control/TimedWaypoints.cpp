#include <mps/planner/ompl/control/TimedWaypoints.h>

using namespace mps::planner::ompl::control;

// TimedWaypoints::TimedWaypoints(const Eigen::Affine3f& tf) :
// _tf(tf){

// }

TimedWaypoints::TimedWaypoints(
   const std::vector<std::pair<float, Eigen::VectorXf> >& waypoints,
                               const Eigen::Affine3f& tf):
                               _tf(tf), _waypoints(waypoints)
{
    if (_waypoints.empty()) {
        throw std::runtime_error("Illegal input arguments. There must be at least one waypoint.");
    }
}


TimedWaypoints::~TimedWaypoints() {

}

Eigen::VectorXf TimedWaypoints::getPosition(float t) {
    Eigen::VectorXf pos;
    getPosition(t, pos);
    return pos;
}

void TimedWaypoints::getPosition(float t, Eigen::VectorXf&  position) const {
    assert(!_waypoints.empty());
    if (_waypoints.size() == 1) {
        return _waypoints.at(0).second();
    }
    // assuming waypoint[0].first == 0.0
    // get first waypoint with wp_t > t
    auto second_wp = std::upper_bound(_waypoints.begin(), _waypoints.end(),
     [](const std::pair<float, Eigen::VectorXf>&a,  const std::pair<float, Eigen::VectorXf>&b){ return a.first < b.first;})
    if (second_wp == _waypoints.end()) { // t is past the duration
        return _waypoints.last()->second;
    }
    auto first_wp = second_wp - 1;
    // interpolate linearly between waypoint positions
    float delta_t = second_wp->first - first_wp->first;
    assert(delta_t > 0.0);
    float rel_t = (t - first_wp->first) / delta_t;
    return rel_t * second_wp->second + (1.0 - rel_t)  * first_wp->second;
}

float TimedWaypoints::getDuration() const {
    return getPreRestDuration() + getRestDuration();
}

float TimedWayppoints::getPreRestDuration() const {
    return _waypoints.last().first ;
}

float TimedWayppoints::getRestDuration() const {
    return _resting_time;
}

void TimedWayppoints::addRestDuration(float dur) {
    _resting_time += dur;
}

void TimedWayppoints::setRestDuration(float dur) {
    _resting_time = dur;
}
