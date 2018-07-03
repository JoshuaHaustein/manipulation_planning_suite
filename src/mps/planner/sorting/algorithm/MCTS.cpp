//
// Created by joshua on 8/14/17.
//
#include <mps/planner/sorting/algorithm/MCTS.h>
#include <mps/planner/util/Logging.h>
#include <mps/planner/pushing/oracle/RampComputer.h>
#include <mps/planner/ompl/control/RampVelocityControl.h>
#include <ompl/datastructures/NearestNeighborsGNAT.h>
#include <queue>
#include <boost/functional/hash.hpp>

namespace logging = mps::planner::util::logging;
namespace ob = ::ompl::base;
namespace oc = ::ompl::control;
namespace mps_state = mps::planner::ompl::state;
namespace mps_control = mps::planner::ompl::control;
using namespace mps::planner::sorting::algorithm;
using namespace mps::planner::ompl::planning::essentials;

////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////// PlanningQuery ///////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
MCTSBase::PlanningQuery::PlanningQuery(ob::State *start_state,
                                       float time_out,
                                       const std::string& robot_name) :
    start_state(start_state),
    robot_name(robot_name),
    time_out(time_out)
{
    stopping_condition = []() {return false;};
    num_control_samples = 10;
}

MCTSBase::PlanningQuery::PlanningQuery(const PlanningQuery &other) = default;

std::string MCTSBase::PlanningQuery::toString() const {
    std::stringstream ss;
    // TODO for completeness should also print target and start state
    ss << "Robot name: " << robot_name << "\n";
    ss << "timeout: " << time_out << "\n";
    ss << "num_control_samples: " << num_control_samples << "\n";
    ss << "\n";
    return ss.str();
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// PlanningBlackboard ////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
MCTSBase::PlanningBlackboard::PlanningBlackboard(PlanningQuery pq) :
        pq(pq),
        robot_id(0)
{
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////// MCTSBase /////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
MCTSBase::MCTSBase(::ompl::control::SpaceInformationPtr si) :
        _si(si),
        _log_prefix("[mps::planner::sorting::algorithm::MCTSBase::")
{
    _state_space = std::dynamic_pointer_cast<mps_state::SimEnvWorldStateSpace>(_si->getStateSpace());
    if (!_state_space) {
        throw std::logic_error(_log_prefix + "setup] Could not cast state space to SimEnvWorldStateSpace");
    }
    // If you need to sample states, you can use a state sampler.
    // _state_sampler = _si->allocStateSampler();
    // _state_sampler = _si->allocValidStateSampler();
    assert(_state_space->getNumObjects() > 1);
    // use this random generator for everything. It allows you to set the seed and make
    // the algorithm behave deterministically according to the seed, if you need.
    _rng = mps::planner::util::random::getDefaultRandomGenerator();
    // TODO initiliaze other members here
}

MCTSBase::~MCTSBase() = default;

void MCTSBase::setup(const PlanningQuery& pq, PlanningBlackboard& blackboard) {
    static const std::string log_prefix(_log_prefix + "setup]");
    // Setup things here that are specific to a planning query.
    setupBlackboard(blackboard);
    logging::logInfo("Planner setup for the following query:\n " + blackboard.pq.toString(), log_prefix);
}

bool MCTSBase::plan(const PlanningQuery& pq,
                            PathPtr path,
                            PlanningStatistics& stats) {
    // Code the general procedure of MCTS in this function
    static const std::string log_prefix(_log_prefix + "plan]");
    PlanningBlackboard blackboard(pq);
    setup(pq, blackboard);
    logging::logInfo("Starting to plan", log_prefix);
    // Here are some examples of how to use OMPL classes
    // First, let's talk about Motions. These are tuples of states and controls
    // and are defined in mps/planner/ompl/planning/Essentials.h. Since a motion
    // can have a parent, you can use motions to represent a search tree.
    // A solution to a planning problem is a path in this search tree, i.e. a sequence of motions.
    // The Motion class is all you need for RRT-based planners. In case of MCTS, I think
    // you can still use it, but you probably need to store some additional information like how
    // many times you explored a branch of the tree etc. Hence, you will probably want to build your own
    // tree data structure on top of this or not use Motions at all. Representing the solution
    // of your planner as a Path, i.e. a sequence of Motions, is useful, however, since it allows
    // you to reuse some code for playback.
    // In any case, you can create a new motion by calling the member function getNewMotion
    MotionPtr my_motion = getNewMotion();
    // You can copy states using the space information -> google OMPL SpaceInformation in namespace ::ompl::control
    _si->copyState(my_motion->getState(), pq.start_state);
    // Similiarly, you can copy controls, or set a control to be null
    _si->nullControl(my_motion->getControl());
    // You can print a state using the printState  member function
    #ifdef DEBUG_PRINTOUTS // this allows you to compile without these print statements
    printState("Sampled state is ", my_motion->getState());
    #endif
    // You can also create new states or controls without using motions
    ::ompl::base::State* a_state = _si->allocState();
    ::ompl::control::Control* a_control = _si->allocControl();
    // These are raw pointers that you will have to delete again -> see end of this function
    // Note that the types are ompl state and control, which are very generic.
    // Since we know that states are of type SimEnvWorldState, we can cast the state
    auto* sim_env_state = dynamic_cast<mps_state::SimEnvWorldState*>(a_state);
    // similarly, you can cast the control (if you know what the control space is)
    auto* ramp_control = dynamic_cast<mps_control::RampVelocityControl*>(a_control);
    // now you can do more specific things with the control and the state.
    // for instance, get the state of object with name "my_object"
    // We first need to know the index of this object
    int idx = _state_space->getObjectIndex("box_2");
    if (idx < 0) {
        // you can log like this. It uses the SimEnv logger under the hood.
        logging::logErr("Could not find object with name box_2", _log_prefix);
        return false;
    }
    // then we can use this index to quickly retrieve its state (without a name lookup)
    auto* object_state = sim_env_state->getObjectState(idx);
    // you can get the configuration (x, y, theta) or its velocity
    Eigen::VectorXf config = object_state->getConfiguration();
    Eigen::VectorXf obj_vel = object_state->getVelocity();
    // Check mps/planning/ompl/SimEnvState.h to learn more about the state spaces.
    // Now let's take a look at the control
    // Above we casted a_control to ramp_control of type RampVelocityControl
    // This is a velocity control, i.e. it specifies velocities for the robot and some duration
    // for which it should apply those. It's a semi=dynamic control, which refers to the idea that
    // the robot and all objects should come to rest again after the action.
    // To guarantee this, a semi-dynamic control is defined such that the robot's velocity is
    // initially 0 and also 0 at the end of the action.
    // It furthermore has a resting time that represents additional time that the robot is waiting for
    // after the action, so that the world can come to rest. See mps::planner::ompl::control::Interfaces.h
    // for more details.
    // There are different ways of achieving such a behaviour. The one that is implemented in this
    // repository is RampVelocityControl. You can use it as follows:
    Eigen::VectorXf vel(3); // a vector of length 3
    vel[0] = 1.0f; // x velocity
    vel[1] = 0.0f; // y velocity
    vel[2] = 0.1f; // theta velocity
    ramp_control->setMaxVelocities(vel, 1.0f); // apply vel for 1s.
    // you can access the resting time through getRestTime() or setRestTime()
    // Ideally, in your algorithm you shouldn't care about many of these underlying details though.
    // For instance, you can just use a RampComputer to compute an action (or a sequence of actions)
    // that move the robot on a straight line from one state to another:
    auto control_space = std::dynamic_pointer_cast<mps_control::RampVelocityControlSpace>(_si->getControlSpace());
    auto robot_state_space = _state_space->getObjectStateSpace(blackboard.robot_id);
    mps::planner::pushing::oracle::RampComputer ramp_computer(robot_state_space->getConfigurationSpace(), control_space);
    std::vector<::ompl::control::Control*> controls; // will contain the computed actions
    // let's get the current robot state
    _si->copyState(sim_env_state, pq.start_state); // copies pq.start_state to sim_env_state
    auto current_robot_state = sim_env_state->getObjectState(blackboard.robot_id);
    // current_robot_state is now the state of the robot in world state sim_env_state
    // next, let's create a new target robot state
    // for this we need the state space of the robot. Note that this is different from the
    // state space of the full world, which is used by _si->allocState();
    auto target_robot_state = dynamic_cast<mps_state::SimEnvObjectState*>(robot_state_space->allocState());
    // let's move the target state relative to the current state
    Eigen::VectorXf robot_config = current_robot_state->getConfiguration();
    robot_config[0] += 0.1; // we want the robot to move 10 cm in x direction
    target_robot_state->setConfiguration(robot_config);
    // now compute the action for that
    ramp_computer.steer(current_robot_state, target_robot_state, controls);
    // you can also check the other interfaces of RampComputer.

    // lastly, we come to the probably MOST IMPORTANT part: PHYSICS SIMULATION
    // You could study the SimEnv.h interface and simulate the outcome of a control yourself, but
    // all of that is already implemented in the SimEnvStatePropatagor.
    // Take a look at mps/planner/ompl/control/SimEnvStatePropagator.h and cpp for more details.
    // We created a propagator in PushSortingPlanner.cpp, so here we only need to retrieve it
    // from the space information:
    auto propagator = std::dynamic_pointer_cast<mps_control::SimEnvStatePropagator>(_si->getStatePropagator());
    // with this the whole physics simulation is simply a call to a single function
    bool valid = propagator->propagate(pq.start_state, ramp_control, sim_env_state);
    // this simulates what happens if you execute the action ramp_control from the world state
    // sim_env_state. The result is stored in sim_env_state
    // The function returns a bool indicating whether during the simulation everything went
    // fine, that is the robot didn't collide with static walls and all objects came to rest
    // The function also takes care of the aforementioned waiting time. That means it sets
    // the waiting time of ramp_control such that all objects come to rest within this waiting time.
    // This only really matters if you have objects that slide like balls though. If you have high
    // fricition the waitting times should usually be around 0.
    // Let's see what the outcome of this action actually is, and print the resulting state
    #ifdef DEBUG_PRINTOUTS // this allows you to compile without these print statements
    printState("Resulting state is ", sim_env_state);
    #endif
    // So as you see, you don't really need to interact much with the simulator. In particular,
    // you will never have to interact with Box2D itself. All of this is abstracted. You should
    // focus here on your planning algorithm and keep things general so that we can apply your algorithm
    // to as many problems as possible.
    // The remainder of this function gives some more structure of how this function could later look
    // like.
    _timer.startTimer(pq.time_out); // start a timer with the specified timeout
    // Do the actual planning
    bool solved = false;
    // loop as long as the problem is not solved, aborted or a timeout occurred
    // the stopping condition can check for instance whether the button in the GUI is still pressed
    while(not _timer.timeOutExceeded() and not pq.stopping_condition() && !solved) {
        blackboard.stats.num_iterations++;
        // TODO implement MCTS here
    }
    blackboard.stats.runtime = _timer.stopTimer();
    blackboard.stats.success = solved;
    // you can stringstream to build strings before printing them
    std::stringstream ss;
    blackboard.stats.print(ss);
    logging::logDebug("Main loop finished, stats:\n" + ss.str(),
                      log_prefix);
    // create the path if we found a solution
    if(solved){
        logging::logInfo("Found a solution", log_prefix);
        // path->initBacktrackMotion(final_motion); // creates a path starting from the last motion
    }
    // clean up - this can become quite tedious, so I suggest you to write some helper functions
    // that allow you to easily reuse states and delete them when they are no longer needed.
    // NOTE that you should NOT call delete on these pointers as this will result in memory leaks.
    // You need to free states and controls using freeState/freeControl functions.
    _si->freeControl(ramp_control);
    _si->freeState(sim_env_state);
    robot_state_space->freeState(target_robot_state);
    for (auto control : controls) {
        _si->freeControl(control);
    }

    logging::logInfo("Planning finished", log_prefix);
    stats = blackboard.stats;
    return solved;
}

void MCTSBase::printState(const std::string& msg, ::ompl::base::State *state) const {
    std::stringstream ss;
    auto* world_state = dynamic_cast<mps_state::SimEnvWorldState*>(state);
    ss.str("");
    ss << msg;
    world_state->print(ss);
    logging::logDebug(ss.str(), "[mps::planner::pushing::oracle::MCTSBase::printState]");
    // see RRT.h and RRT.cpp to see how you could also use the GUI to visualize a state
}

MotionPtr MCTSBase::getNewMotion() {
    if (not _motions_cache.empty()) {
        MotionPtr ptr = _motions_cache.top();
        _motions_cache.pop();
        return ptr;
    }
    return std::make_shared<Motion>(_si);
}

void MCTSBase::cacheMotion(MotionPtr ptr) {
    _motions_cache.push(ptr);
}

void MCTSBase::setupBlackboard(PlanningBlackboard &pb) {
    pb.robot_id = 0;
    {
        int tmp_robot_id = _state_space->getObjectIndex(pb.pq.robot_name);
        if (tmp_robot_id < 0) {
            throw std::logic_error("[mps::planner::pushing::oracle::MCTSBase::setupBlackboard]"
                    "Could not retrieve id for robot " + pb.pq.robot_name);
        }
        pb.robot_id = (unsigned int)tmp_robot_id;
    }
}

////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////// DeterministicMCTSBase /////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
DeterministicMCTS::DeterministicMCTS(::ompl::control::SpaceInformationPtr si) :
        MCTSBase(si)
{
}

DeterministicMCTS::~DeterministicMCTS() = default;

void DeterministicMCTS::setup(const PlanningQuery& pq, PlanningBlackboard& blackboard) {
    MCTSBase::setup(pq, blackboard);
}

////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// NonDeterministicMCTS /////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////
NonDeterministicMCTS::NonDeterministicMCTS(::ompl::control::SpaceInformationPtr si) :
        MCTSBase(si)
{
}

NonDeterministicMCTS::~NonDeterministicMCTS() = default;

void NonDeterministicMCTS::setup(const PlanningQuery& pq, PlanningBlackboard& blackboard)
{
    MCTSBase::setup(pq, blackboard);
}