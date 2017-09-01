//
// Created by joshua on 8/14/17.
//

#include <mps/planner/pushing/algorithm/RRT.h>
#include <mps/planner/util/Logging.h>
#include <mps/planner/ompl/control/Interfaces.h>
#include <ompl/datastructures/NearestNeighborsSqrtApprox.h>

#include <queue>

namespace logging = mps::planner::util::logging;
namespace ob = ::ompl::base;
namespace oc = ::ompl::control;
namespace mps_state = mps::planner::ompl::state;
namespace mps_control = mps::planner::ompl::control;
using namespace mps::planner::pushing::algorithm;

SemiDynamicRRT::PlanningQuery::PlanningQuery(std::shared_ptr<ob::GoalSampleableRegion> goal_region,
                                             ob::State *start_state, float time_out) :
    goal_region(goal_region),
    start_state(start_state),
    time_out(time_out)
{
    stopping_condition = []() {return true;};
    goal_bias = 0.1f;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////// Motion ////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
SemiDynamicRRT::Motion::Motion(oc::SpaceInformationPtr si) {
    _state = si->allocState();
    _control = si->allocControl();
    _weak_si = si;
    _parent = nullptr;
}

SemiDynamicRRT::Motion::Motion(const Motion &other) {
    _weak_si = other._weak_si;
    auto si = _weak_si.lock();
    if (!si) {
        std::logic_error("[mps::planner::algorithm::SemiDynamicRRT::Motion::Motion(const Motion&)]"
                                 " Could not access space information. The other motion is invalid.");
    }
    si->copyState(_state, other._state);
    si->copyControl(_control, other._control);
    _parent = other._parent;
}

SemiDynamicRRT::Motion::~Motion() {
    auto si = _weak_si.lock();
    if (!si) {
        logging::logErr("Could not access space information. This will result in memory leaked.",
                        "[mps::planner::algorithm::SemiDynamicRRT::Motion::~Motion]");
        return;
    }
    if (_control) {
        si->freeControl(_control);
    }
    if (_state) {
        si->freeState(_state);
    }
}

SemiDynamicRRT::Motion& SemiDynamicRRT::Motion::operator=(const SemiDynamicRRT::Motion& other) {
    _weak_si = other._weak_si;
    auto si = _weak_si.lock();
    if (!si) {
        std::logic_error("[mps::planner::algorithm::SemiDynamicRRT::Motion::operator=]"
                                 " Could not access space information. The other motion is invalid.");
    }
    si->copyState(_state, other._state);
    si->copyControl(_control, other._control);
    _parent = other._parent;
    return *this;
}

ob::State* SemiDynamicRRT::Motion::getState() {
    return _state;
}

oc::Control* SemiDynamicRRT::Motion::getControl() {
    return _control;
}

SemiDynamicRRT::MotionPtr SemiDynamicRRT::Motion::getParent() {
    return _parent;
}

SemiDynamicRRT::MotionConstPtr SemiDynamicRRT::Motion::getConstParent() {
    return _parent;
}

void SemiDynamicRRT::Motion::setParent(MotionPtr parent) {
    _parent = parent;
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////// Path //////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
SemiDynamicRRT::Path::Path(oc::SpaceInformationPtr si) :
    ::ompl::base::Path(si)
{
    _sic = si;
}

SemiDynamicRRT::Path::~Path() = default;

double SemiDynamicRRT::Path::length() const {
    return _length;
}

::ompl::base::Cost SemiDynamicRRT::Path::cost(const ob::OptimizationObjectivePtr& oo) const {
    throw std::logic_error("[mps::planner::algorithm::SemiDynamicRRT::Path::cost] Not implemented");
}

bool SemiDynamicRRT::Path::check() const {
    for (auto motion : _motions) {
        if (not _sic->isValid(motion->getState())) {
            return false;
        }
    }
    return true;
}

void SemiDynamicRRT::Path::print(std::ostream &out) const {
    out << "SemiDynamicRRT::Path: /n";
    for (auto motion : _motions) {
        out << "State: ";
        motion->getState()->as<mps_state::SimEnvWorldState>()->print(out);
        out << " Control: ";
        auto control = dynamic_cast<mps_control::SemiDynamicVelocityControl*>(motion->getControl());
        if (!control) {
            throw std::logic_error("[mps::planner::pushing::SemiDynamicRRT::Path::print] Could not cast control to SemiDynamicVelocityControl");
        }
        control->serializeInNumbers(out);
        out << "/n";
    }
}

void SemiDynamicRRT::Path::append(MotionPtr motion) {
    if (_motions.size() > 0) {
        _length += _sic->distance(_motions.at(_motions.size() - 1)->getState(), motion->getState());
    }
    _motions.push_back(motion);
}

void SemiDynamicRRT::Path::initBacktrackMotion(MotionPtr motion) {
    clear();
    _motions.push_back(motion);
    MotionPtr parent_motion = motion->getParent();
    while (parent_motion != nullptr) {
        _motions.push_back(parent_motion);
        parent_motion = parent_motion->getParent();
    }
    std::reverse(_motions.begin(), _motions.end());
}

void SemiDynamicRRT::Path::clear() {
    _motions.clear();
}

unsigned int SemiDynamicRRT::Path::getNumMotions() const {
    return _motions.size();
}

SemiDynamicRRT::MotionPtr SemiDynamicRRT::Path::getMotion(unsigned int i) {
    return _motions.at(i);
}

SemiDynamicRRT::MotionConstPtr SemiDynamicRRT::Path::getConstMotion(unsigned int i) const {
    return _motions.at(i);
}

SemiDynamicRRT::SemiDynamicRRT(::ompl::control::SpaceInformationPtr si) :
        _si(si),
        _log_prefix("[mps::planner::pushing::algorithm::SemiDynamicRRT::"),
        _is_setup(false)
{
    auto state_space = std::dynamic_pointer_cast<mps_state::SimEnvWorldStateSpace>(si->getStateSpace());
    if (!state_space) {
        throw std::logic_error(_log_prefix + "SemiDynamicRRT] Could not cast state space to SimEnvWorldStateSpace");
    }
    _distance_measure = std::make_shared<PushPlannerDistanceMeasure>(state_space);
}

SemiDynamicRRT::~SemiDynamicRRT() = default;

void SemiDynamicRRT::setup() {
    // TODO set everything up
    if (!_tree) {
        // TODO this is not an efficient data structure, but we need to have a structure that allows modifying the distance function
        // TODO or a data structure for each setting of active objects (in rearrangement planning context)
        _tree = std::make_shared<::ompl::NearestNeighborsSqrtApprox< MotionPtr > >();
    }
    _is_setup = true;
    _control_sampler = _si->allocDirectedControlSampler();
    _state_sampler = _si->allocStateSampler();
    _rng = mps::planner::util::random::getDefaultRandomGenerator();
}

bool SemiDynamicRRT::plan(const PlanningQuery& pq, Path& path) {
    static const std::string log_prefix(_log_prefix + "plan]");
    logging::logDebug("Starting to plan", log_prefix);

    if (not _is_setup) {
        logging::logWarn("Planner is not setup, aborting", log_prefix);
        return false;
    }

    MotionPtr current_motion = getNewMotion();
    MotionPtr sample = getNewMotion();
    MotionPtr final_motion = nullptr;
    // set the state to be the start state
    _si->copyState(current_motion->getState(), pq.start_state);
    // set the control to be null
    _si->nullControl(current_motion->getControl());
    // Initialize the tree
    _tree.reset();
    _tree->add(current_motion);
    final_motion = current_motion;

    bool solved = pq.goal_region->isSatisfied(current_motion->getState());
    logging::logDebug("Entering main loop", log_prefix);
    // Do the actual planning
    while(not pq.stopping_condition() && !solved) {
        // sample random state with goal biasing
        if( _rng->uniform01() < pq.goal_bias && pq.goal_region->canSample()){
            pq.goal_region->sampleGoal(sample->getState());
        }else{
            _state_sampler->sampleUniform(sample->getState());
        }
        // Sample which object should be active
        unsigned int object_id = (unsigned int)(_rng->uniformInt(0, _num_objects));
        _distance_measure->setAll(false);
        _distance_measure->setActive(object_id, true);
        // Get nearest tree node
        logging::logDebug("Searching for nearest neighbor.", log_prefix);
        current_motion = _tree->nearest(sample);
        logging::logDebug("Found nearest neighbor", log_prefix);
        // Compute action to take us towards the sampled state
        MotionPtr new_motion = getNewMotion();
        _control_sampler->sampleTo(new_motion->getControl(),
                                   current_motion->getState(),
                                   sample->getState());
        // TODO this implies we might propagate a control twice
        _state_propagator->propagate(current_motion->getState(),
                                     new_motion->getControl(),
                                     new_motion->getState());
        if (_si->isValid(new_motion->getState())) {
            new_motion->setParent(current_motion);
            _tree->add(new_motion);
            solved = pq.goal_region->isSatisfied(new_motion->getState());
            final_motion = new_motion;
        } else {
            cacheMotion(new_motion);
        }
    }

    logging::logDebug("Main loop finished", log_prefix);

    // create the path if we found a solution
    if(solved){
        logging::logInfo("Found a solution", log_prefix);
        path.initBacktrackMotion(final_motion);
    }

    // clean up
     _tree->clear();

    logging::logInfo("Planning finished", log_prefix);
    return solved;
}

SemiDynamicRRT::MotionPtr SemiDynamicRRT::getNewMotion() {
    if (not _motions_cache.empty()) {
        MotionPtr ptr = _motions_cache.top();
        _motions_cache.pop();
        return ptr;
    }
    return std::make_shared<Motion>(_si);
}

void SemiDynamicRRT::cacheMotion(MotionPtr ptr) {
    _motions_cache.push(ptr);
}
