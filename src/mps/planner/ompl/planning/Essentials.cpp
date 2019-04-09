//
// Created by joshua on 9/5/17.
//

#include <mps/planner/ompl/control/Interfaces.h>
#include <mps/planner/ompl/planning/Essentials.h>
#include <mps/planner/ompl/state/SimEnvState.h>
#include <mps/planner/util/Logging.h>

namespace oc = ompl::control;
namespace ob = ompl::base;
namespace mps_state = mps::planner::ompl::state;
namespace mps_control = mps::planner::ompl::control;
using namespace mps::planner::ompl::planning::essentials;
using namespace mps::planner::util;

////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////// Motion ////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
Motion::Motion(oc::SpaceInformationPtr si)
{
    _state = si->allocState();
    _control = si->allocControl();
    _weak_si = si;
    _parent = nullptr;
}

Motion::Motion(const Motion& other)
{
    _weak_si = other._weak_si;
    auto si = _weak_si.lock();
    if (!si) {
        std::logic_error("[mps::planner::ompl::planning::essentials::Motion::Motion(const Motion&)]"
                         " Could not access space information. The other motion is invalid.");
    }
    si->copyState(_state, other._state);
    si->copyControl(_control, other._control);
    _parent = other._parent;
}

Motion::~Motion()
{
    auto si = _weak_si.lock();
    if (!si) {
        logging::logErr("Could not access space information. This will result in memory leaked.",
            "[mps::planner::ompl::planning::essentials::Motion::~Motion]");
        return;
    }
    if (_control) {
        si->freeControl(_control);
    }
    if (_state) {
        si->freeState(_state);
    }
}

Motion& Motion::operator=(const Motion& other)
{
    _weak_si = other._weak_si;
    auto si = _weak_si.lock();
    if (!si) {
        std::logic_error("[mps::planner::ompl::planning::essentials::Motion::operator=]"
                         " Could not access space information. The other motion is invalid.");
    }
    si->copyState(_state, other._state);
    si->copyControl(_control, other._control);
    _parent = other._parent;
    return *this;
}

ob::State* Motion::getState()
{
    return _state;
}

ob::State const* Motion::getConstState() const
{
    return _state;
}

oc::Control* Motion::getControl()
{
    return _control;
}

oc::Control const* Motion::getConstControl() const
{
    return _control;
}

MotionPtr Motion::getParent()
{
    return _parent;
}

MotionConstPtr Motion::getConstParent() const
{
    return _parent;
}

void Motion::setParent(MotionPtr parent)
{
    _parent = parent;
}

void Motion::reset()
{
    _parent = nullptr;
}

MotionPtr Motion::deepCopy() const
{
    auto si = _weak_si.lock();
    if (!si) {
        throw std::logic_error("Could not copy Motion. Space Information does no longer exist.");
    }
    auto new_motion = std::make_shared<Motion>(si);
    si->copyState(new_motion->getState(), _state);
    si->copyControl(new_motion->getControl(), _control);
    return new_motion;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////// Path //////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
Path::Path(oc::SpaceInformationPtr si)
    : ::ompl::base::Path(si)
{
    _sic = si;
}

Path::~Path() = default;

double Path::length() const
{
    return _length;
}

::ompl::base::Cost Path::cost(const ob::OptimizationObjectivePtr& oo) const
{
    throw std::logic_error("[mps::planner::ompl::planning::essentials::Path::cost] Not implemented");
}

bool Path::check() const
{
    for (auto motion : _motions) {
        if (not _sic->isValid(motion->getState())) {
            return false;
        }
    }
    return true;
}

void Path::print(std::ostream& out) const
{
    out << "Path: /n";
    for (auto motion : _motions) {
        out << "State: ";
        // TODO cast these states to some interface that allows printing.
        // TODO it's unnecessary to restrict this class to SimEnvWorldStates and SemiDYnamicVelocityControl
        motion->getState()->as<mps_state::SimEnvWorldState>()->print(out);
        out << " Control: ";
        auto control = dynamic_cast<mps_control::SemiDynamicVelocityControl*>(motion->getControl());
        if (!control) {
            throw std::logic_error("[mps::planner::ompl::planning::essentials::Path::print] Could not cast control to SemiDynamicVelocityControl");
        }
        control->serializeInNumbers(out);
        out << "/n";
    }
}

void Path::append(MotionPtr motion)
{
    if (_motions.size() > 0) {
        _length += _sic->distance(_motions.at(_motions.size() - 1)->getState(), motion->getState());
    }
    _motions.push_back(motion);
}

void Path::concat(std::shared_ptr<Path> other, unsigned int n)
{
    unsigned int last_wp_id = std::min(other->getNumMotions(), n);
    for (unsigned int i = 0; i < last_wp_id; ++i) {
        append(other->getMotion(i));
    }
}

void Path::initBacktrackMotion(MotionPtr motion)
{
    clear();
    _motions.push_back(motion);
    MotionPtr parent_motion = motion->getParent();
    while (parent_motion != nullptr) {
        _motions.push_back(parent_motion);
        parent_motion = parent_motion->getParent();
    }
    std::reverse(_motions.begin(), _motions.end());
}

PathPtr Path::getSubPath(unsigned int s) const
{
    auto new_path = std::make_shared<Path>(_sic);
    for (unsigned int i = s; i < _motions.size(); ++i) {
        new_path->append(_motions.at(i));
    }
    return new_path;
}

void Path::clear()
{
    _motions.clear();
}

unsigned int Path::getNumMotions() const
{
    return _motions.size();
}

MotionPtr Path::getMotion(unsigned int i)
{
    return _motions.at(i);
}

MotionPtr Path::last()
{
    return _motions.at(getNumMotions() - 1);
}

MotionConstPtr Path::getConstMotion(unsigned int i) const
{
    return _motions.at(i);
}

PathPtr Path::deepCopy() const
{
    PathPtr new_path = std::make_shared<Path>(_sic);
    for (auto motion : _motions) {
        auto new_motion = motion->deepCopy();
        new_path->append(new_motion);
    }
    return new_path;
}

CostFunction::~CostFunction() = default;

double CostFunction::cost(PathPtr path, int limit)
{
    double value = 0.0;
    unsigned int num_motions = path->getNumMotions();
    if (limit >= 0)
        num_motions = std::min((unsigned int)limit + 1, num_motions);
    if (num_motions <= 1)
        return 0.0;
    auto prev_motion = path->getMotion(0);
    // run over path and accumulate cost
    for (unsigned int i = 0; i < num_motions; ++i) {
        auto current_motion = path->getMotion(i);
        value += cost(prev_motion, current_motion);
        prev_motion = current_motion;
    }
    return value;
}