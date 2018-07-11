//
// Created by joshua on 8/14/17.
//

#ifndef MANIPULATION_PLANNING_SUITE_MCTS_H
#define MANIPULATION_PLANNING_SUITE_MCTS_H

// OMPL includes
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/base/State.h>
#include <ompl/base/Path.h>
#include <ompl/datastructures/NearestNeighbors.h>
// MPS includes
#include <mps/planner/ompl/control/SimEnvStatePropagator.h>
#include <mps/planner/ompl/planning/Essentials.h>
#include <mps/planner/ompl/state/SimEnvWorldStateDistanceMeasure.h>
#include <mps/planner/ompl/state/goal/ObjectsRelocationGoal.h>
#include <mps/planner/util/Random.h>
#include <mps/planner/util/Time.h>
#include <mps/planner/util/Logging.h>
#include <mps/sdf/SDF.h>
// stl
#include <stack>
#include <mps/planner/ompl/control/NaiveControlSampler.h>
#include <mps/planner/pushing/oracle/OracleControlSampler.h>
#include <iomanip>

namespace mps {
    namespace planner {
        namespace sorting {
            namespace algorithm {
                // class DebugDrawer;
                // typedef std::shared_ptr<DebugDrawer> DebugDrawerPtr;
                // typedef std::weak_ptr<DebugDrawer> DebugDrawerWeakPtr;

                struct PlanningStatistics {
                    // add flags here for interesting statistics
                    unsigned int num_iterations; // number of iterations
                    float runtime; // total runtime of the planner
                    bool success; // whether the planner found a solution or not

                    // Print function for human readable printing
                    void print(std::ostream& os) const{
                        os << "Planning statistics: \n";
                        os << "     runtime: " << std::fixed << runtime << std::setprecision(3);
                        os << "     num_iterations: " << num_iterations;
                        os << "     success: " << success;
                        os << std::endl;
                    }

                    // Prints the header of a cvs file
                    void printCVSHeader(std::ostream& os) const {
                        os << "runtime,num_iterations,success" << std::endl;
                    }

                    // Print the stats in cvs format
                    void printCVS(std::ostream& os) const {
                        os << runtime << ", "
                           << num_iterations << ", "
                           << success << ", "
                           << std::endl;
                    }

                    // Read stats from input stream
                    void readCVS(std::istream& is) {
                        std::vector<std::string> splits;
                        std::string line;
                        std::getline(is, line);
                        mps::planner::util::serialize::splitString(line, splits);
                        if (splits.size() != 3) { // number of stats items
                            mps::planner::util::logging::logErr("Could not read stats.", "[PlanningStatistics::readCVS]");
                        }
                        runtime = std::stof(splits[0]);
                        num_iterations = std::stoi(splits[1]);
                        success = std::stoul(splits[2]);
                    }

                    std::string to_string() const{
                        std::stringstream ss;
                        print(ss);
                        return ss.str();
                    }

                    PlanningStatistics() {
                        runtime = 0.0;
                        num_iterations = 0;
                        success = false;
                        // add initialization of more statistics here
                    }
                };

                /*
                 * This class serves as base for your algorithms. You should encapsulate
                 * all functionality that is common for different variants of MCTS in this
                 * class and provide specializations in the following subclasses.
                 * This allows you to reuse a lot of code without copy-paste and also allows
                 * you to swap out the actual planning algorithm easily.
                 */
                class MCTSBase {
                public:

                    // A PlanningQuery contains all information the algorithm needs to plan.
                    struct PlanningQuery {
                        ::ompl::base::State* start_state; // start state of the problem (SimEnvWorldStateSpace)
                        std::string robot_name;
                        float time_out; // time out in seconds
                        unsigned int num_control_samples; // number of control samples
                        std::function<bool()> stopping_condition; // optionally a customized stopping condition
                        std::vector<unsigned int> groups; // an array saving the group for each object id. NOTE that this array also includes an entry for the robot, which should be ignored
                        PlanningQuery(::ompl::base::State* start_state,
                                      float time_out,
                                      std::vector<unsigned int>& groups,
                                      const std::string& robot_name);
                        PlanningQuery(const PlanningQuery& other);
                        std::string toString() const;
                    };


                protected:
                    // a struct that keeps all planning relevant information
                    struct PlanningBlackboard {
                        PlanningQuery pq;
                        PlanningStatistics stats;
                        unsigned int robot_id;
                        explicit PlanningBlackboard(PlanningQuery pq);
                    };

                public:
                    /**
                     * Creates a new instance of MCTSBase.
                     * The argument si contains the control (action) space, state space
                     * and the state propagator.
                     */
                    MCTSBase(::ompl::control::SpaceInformationPtr si);
                    virtual ~MCTSBase() = 0; // destructor
                    /*
                     * Execute the algorithm for the given query. The solution is saved in
                     * the path argument and statistics about the planning process are recorded
                     * in stats. The function returns true if the planning was successful.
                     */
                    virtual bool plan(const PlanningQuery& pq,
                                      mps::planner::ompl::planning::essentials::PathPtr path,
                                      PlanningStatistics& stats);
                    // TODO: Define functions here that are common for MCTS algorithm
                protected:
                    // set up things
                    virtual void setup(const PlanningQuery& pq, PlanningBlackboard& blackboard);
                    // helper function to print a state
                    void printState(const std::string& msg, ::ompl::base::State* state) const;
                    // helper functions for caching motions. Motions are tuples of state and control(action).
                    mps::planner::ompl::planning::essentials::MotionPtr getNewMotion();
                    void cacheMotion(mps::planner::ompl::planning::essentials::MotionPtr ptr);

                    // member variables that descended classes should be able to access
                    ::ompl::control::SpaceInformationPtr _si;
                    // ::ompl::base::ValidStateSamplerPtr _state_sampler;
                    mps::planner::ompl::state::SimEnvWorldStateSpacePtr _state_space;
                    ::ompl::RNGPtr _rng;
                    mps::planner::util::time::Timer _timer;

                private:
                    void setupBlackboard(PlanningBlackboard& pb);

                    std::string _log_prefix;
                    std::stack<mps::planner::ompl::planning::essentials::MotionPtr> _motions_cache;
                };

                typedef std::shared_ptr<MCTSBase> MCTSBasePtr;
                typedef std::shared_ptr<const MCTSBase> MCTSBaseConstPtr;
                typedef std::weak_ptr<MCTSBase> MCTSBaseWeakPtr;
                typedef std::weak_ptr<const MCTSBase> MCTSBaseWeakConstPtr;

                /**
                 * In this class you can implement your first algorithm to address the sorting problem.
                 * As we agreed you should start with a deterministic Monte Carlo Tree search.
                 */
                class DeterministicMCTS : public MCTSBase {
                public:
                    DeterministicMCTS(::ompl::control::SpaceInformationPtr si);
                    ~DeterministicMCTS() override;
                protected:
                    void setup(const PlanningQuery& pq, PlanningBlackboard& blackboard) override;
                    // TODO implement functions for deterministic MCTS here
                private:
                    // TODO place private menbers here
                };
                typedef std::shared_ptr<DeterministicMCTS> DeterministicMCTSPtr;
                typedef std::shared_ptr<const DeterministicMCTS> DeterministicMCTSConstPtr;
                typedef std::weak_ptr<DeterministicMCTS> DeterministicMCTSWeakPtr;
                typedef std::weak_ptr<const DeterministicMCTS> DeterministicMCTSWeakConstPtr;

                /**
                 *  In this class you could implement a non-deterministic MCTS algorithm.
                 *  Depending on how well Deterministic MCTS is working, this might not be necessary.
                 */
                class NonDeterministicMCTS : public MCTSBase {
                public:
                    NonDeterministicMCTS(::ompl::control::SpaceInformationPtr si);
                    ~NonDeterministicMCTS() override;
                protected:
                    void setup(const PlanningQuery& pq, PlanningBlackboard& blackboard) override;
                    // TODO implement functions for deterministic MCTS here
                private:
                    // TODO place private menbers here
                };
                typedef std::shared_ptr<NonDeterministicMCTS> NonDeterministicMCTSPtr;
                typedef std::shared_ptr<const NonDeterministicMCTS> NonDeterministicMCTSConstPtr;
                typedef std::weak_ptr<NonDeterministicMCTS> NonDeterministicMCTSWeakPtr;
                typedef std::weak_ptr<const NonDeterministicMCTS> NonDeterministicMCTSWeakConstPtr;
            }
        }
    }
}
#endif //MANIPULATION_PLANNING_SUITE_RRT_H
