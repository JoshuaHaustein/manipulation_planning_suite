//
// Created by joshua on 03/21/19.
//

#ifndef MPS_ALGORITHM_INTERFACES_H
#define MPS_ALGORITHM_INTERFACES_H
// ompl includes
#include <ompl/datastructures/NearestNeighbors.h>
// stl includes
#include <iomanip>
#include <memory>
#include <stack>
#include <string>
#include <vector>
// sim_env includes
#include <sim_env/SimEnv.h>
// mps includes
#include <mps/planner/ompl/planning/Essentials.h>
#include <mps/planner/ompl/state/SimEnvState.h>
#include <mps/planner/ompl/state/SimEnvWorldStateDistanceMeasure.h>
#include <mps/planner/ompl/state/goal/ObjectsRelocationGoal.h>
#include <mps/planner/util/Logging.h>
#include <mps/planner/util/Serialize.h>
#include <mps/planner/util/Time.h>

namespace mps {
namespace planner {
    namespace pushing {
        namespace algorithm {
            // Forward declarations
            class DebugDrawer;
            typedef std::shared_ptr<DebugDrawer> DebugDrawerPtr;
            typedef std::weak_ptr<DebugDrawer> DebugDrawerWeakPtr;

            class SliceDrawerInterface;
            typedef std::shared_ptr<SliceDrawerInterface> SliceDrawerInterfacePtr;

            struct PlanningStatistics {
                unsigned int num_iterations;
                unsigned int num_state_propagations;
                unsigned int num_samples;
                unsigned int num_nearest_neighbor_queries;
                float runtime;
                bool success;
                float cost_before_shortcut;
                float cost_after_shortcut;
                bool reproducible;
                bool reproducible_after_shortcut;

                void print(std::ostream& os) const
                {
                    os << "Planning statistics: \n";
                    os << "     num_iterations: " << num_iterations;
                    os << "     num_state_propagations: " << num_state_propagations;
                    os << "     num_samples: " << num_samples;
                    os << "     num_nearest_neighbor_queries: " << num_nearest_neighbor_queries;
                    os << "     run_time: " << std::fixed << runtime << std::setprecision(3);
                    os << "     success: " << success;
                    os << "     reproducible: " << reproducible;
                    os << "     cost_before_shortcut: " << cost_before_shortcut << std::setprecision(3);
                    os << "     cost_after_shortcut: " << cost_after_shortcut << std::setprecision(3);
                    os << "     reproducible_after_shortcut: " << reproducible_after_shortcut;
                    os << std::endl;
                }

                void printCVSHeader(std::ostream& os) const
                {
                    os << "#iterations,#propagations,#samples,#nn_queries,runtime,success,reproducible,cost_before_shortcut,cost_after_shortcut,reproducible_after_shortcut" << std::endl;
                }

                void printCVS(std::ostream& os) const
                {
                    os << num_iterations << ", "
                       << num_state_propagations << ", "
                       << num_samples << ", "
                       << num_nearest_neighbor_queries << ", "
                       << runtime << ", "
                       << success << ", "
                       << reproducible << ", "
                       << cost_before_shortcut << ", "
                       << cost_after_shortcut << ", "
                       << reproducible_after_shortcut << std::endl;
                }

                void readCVS(std::istream& is)
                {
                    std::vector<std::string> splits;
                    std::string line;
                    std::getline(is, line);
                    mps::planner::util::serialize::splitString(line, splits);
                    if (splits.size() != 10) {
                        mps::planner::util::logging::logErr("Could not read stats.", "[PlanningStatistics::readCVS]");
                    }
                    num_iterations = std::stoul(splits[0]);
                    num_state_propagations = std::stoul(splits[1]);
                    num_samples = std::stoul(splits[2]);
                    num_nearest_neighbor_queries = std::stoul(splits[3]);
                    runtime = std::stof(splits[4]);
                    success = std::stoul(splits[5]);
                    reproducible = std::stoul(splits[6]);
                    cost_before_shortcut = std::stof(splits[7]);
                    cost_after_shortcut = std::stof(splits[8]);
                    reproducible_after_shortcut = std::stoul(splits[9]);
                }

                std::string to_string() const
                {
                    std::stringstream ss;
                    print(ss);
                    return ss.str();
                }

                PlanningStatistics()
                {
                    num_nearest_neighbor_queries = 0;
                    num_iterations = 0;
                    num_state_propagations = 0;
                    num_samples = 0;
                    runtime = 0.0;
                    success = false;
                    reproducible = false;
                    cost_before_shortcut = 0.0f;
                    cost_after_shortcut = 0.0f;
                    reproducible_after_shortcut = false;
                }
            };

            class RearrangementPlanner {
            public:
                struct PlanningQuery {
                    ompl::state::goal::ObjectsRelocationGoalPtr goal_region; // goal region
                    mps::planner::ompl::state::SimEnvWorldState* start_state; // start state of the problem (SimEnvWorldStateSpace)
                    std::string robot_name;
                    float time_out; // time out in seconds
                    std::function<bool()> stopping_condition; // optionally a customized stopping condition
                    mps::planner::ompl::planning::essentials::PathPtr path; // stores the path
                    ::ompl::base::ParamSetPtr parameters; // stores parameters for the planner - intialized with default values in createPlanningQuery(..)
                    std::vector<float> weights; // optional weights for the distance functions
                    PlanningQuery(const PlanningQuery& other);
                    std::string toString() const;

                protected:
                    friend class RearrangementPlanner;
                    // use factory method create PlanningQuery to obtain an instance of a planning query
                    PlanningQuery(ompl::state::goal::ObjectsRelocationGoalPtr goal_region,
                        mps::planner::ompl::state::SimEnvWorldState* start_state,
                        const std::string& robot_name,
                        float time_out);
                };
                typedef std::shared_ptr<PlanningQuery> PlanningQueryPtr;

                RearrangementPlanner(::ompl::control::SpaceInformationPtr si);
                virtual ~RearrangementPlanner() = 0;

                /**
                 * Plan a rearrangement solution for the given planning query. If a solution is found,
                 * it will be stored in pq.path.
                 * @params pq - planning query
                 */
                bool plan(PlanningQueryPtr pq);
                /**
                 * Plan a rearrangement solution for the given planning query. If a solution is found,
                 * it will be stored in pq.path.
                 * @params pq - planning query
                 * @param stats - struct to record planning statistics
                 */
                virtual bool plan(PlanningQueryPtr pq, PlanningStatistics& stats) = 0;

                /**
                 * Factory method to construct a new planning query. This returns a planning query that can 
                 * be modified before calling plan(..). Use this function to create a planning query.
                 */
                virtual PlanningQueryPtr createPlanningQuery(mps::planner::ompl::state::goal::ObjectsRelocationGoalPtr goal_region,
                    mps::planner::ompl::state::SimEnvWorldState* start_state, const std::string& robot_name, float timeout = 60.0);

                void setDebugDrawer(DebugDrawerPtr debug_drawer);

            protected:
                struct PlanningBlackboard {
                    PlanningQueryPtr pq;
                    PlanningStatistics stats;
                    unsigned int robot_id;
                    explicit PlanningBlackboard(PlanningQueryPtr pq);
                };
                void setupBlackboard(PlanningBlackboard& pb);
                void printState(const std::string& msg, ::ompl::base::State* state) const;
                // member variables
                ::ompl::control::SpaceInformationPtr _si;
                std::shared_ptr<mps::planner::util::time::Timer> _timer;
                mps::planner::ompl::state::SimEnvWorldStateSpacePtr _state_space;
                DebugDrawerPtr _debug_drawer;
            };
            typedef std::shared_ptr<RearrangementPlanner> RearrangementPlannerPtr;
            typedef std::shared_ptr<const RearrangementPlanner> RearrangementPlannerConstPtr;
            typedef std::weak_ptr<RearrangementPlanner> RearrangementPlannerWeakPtr;
            typedef std::weak_ptr<const RearrangementPlanner> RearrangementPlannerConstWeakPtr;

            /***
             *  A slice of the joint configuration space of robot and objects.
             */
            struct Slice {
                typedef std::function<double(const mps::planner::ompl::planning::essentials::MotionPtr&,
                    const mps::planner::ompl::planning::essentials::MotionPtr&)>
                    StateDistanceFn;
                Slice(mps::planner::ompl::planning::essentials::MotionPtr repr, StateDistanceFn distance_fn);
                ~Slice();
                void addSample(ompl::planning::essentials::MotionPtr motion);
                void clear();
                void reset(ompl::planning::essentials::MotionPtr repr);
                std::shared_ptr<::ompl::NearestNeighbors<mps::planner::ompl::planning::essentials::MotionPtr>> slice_samples_nn;
                std::vector<ompl::planning::essentials::MotionPtr> slice_samples_list;
                mps::planner::ompl::planning::essentials::MotionPtr repr; // representative
            };
            typedef std::shared_ptr<Slice> SlicePtr;
            typedef std::shared_ptr<const Slice> SliceConstPtr;

            // A distance function defined on motions between sim env states
            // that only takes the distance of the robot into account
            struct RobotStateDistanceFn {
                ompl::state::SimEnvWorldStateDistanceProjection distance_measure;
                void setRobotId(unsigned int id);
                double distance(const ompl::planning::essentials::MotionPtr& motion_a,
                    const ompl::planning::essentials::MotionPtr& motion_b) const;
                RobotStateDistanceFn(ompl::state::SimEnvWorldStateSpacePtr state_space);
            };
            typedef std::shared_ptr<RobotStateDistanceFn> RobotStateDistanceFnPtr;

            // A distance function defined on motions between sim env states
            // that only takes the distance of movable objects into account
            struct ObjectArrangementDistanceFn {
                ompl::state::SimEnvWorldStateDistanceMeasure distance_measure;
                void setRobotId(unsigned int id);
                double distance(const SliceConstPtr& slice_a, const SliceConstPtr& slice_b) const;
                explicit ObjectArrangementDistanceFn(ompl::state::SimEnvWorldStateSpacePtr state_space,
                    const std::vector<float>& weights = std::vector<float>());
            };
            typedef std::shared_ptr<ObjectArrangementDistanceFn> ObjectArrangementDistanceFnPtr;

            /***
             *  MotionCache maintains a list of motions to save reallocation.
             * The MotionType must have a constructor of the form MotionType(::ompl::control::SpaceInformationPtr).
             */
            template <typename MotionType = mps::planner::ompl::planning::essentials::Motion>
            class MotionCache {
            public:
                MotionCache(::ompl::control::SpaceInformationPtr si)
                    : _si(si)
                {
                }
                ~MotionCache() = default;

                std::shared_ptr<MotionType> getNewMotion()
                {
                    if (not _motions_cache.empty()) {
                        auto ptr = _motions_cache.top();
                        _motions_cache.pop();
                        return ptr;
                    }
                    return std::make_shared<MotionType>(_si);
                }

                void cacheMotion(std::shared_ptr<MotionType> ptr)
                {
                    _motions_cache.push(ptr);
                }

                void cacheMotions(std::vector<std::shared_ptr<MotionType>>& motions)
                {
                    for (auto& motion : motions) {
                        _motions_cache.push(motion);
                    }
                    motions.clear();
                }

            private:
                std::stack<std::shared_ptr<MotionType>> _motions_cache;
                ::ompl::control::SpaceInformationPtr _si;
            };

            class SliceCache {
            public:
                SliceCache(RobotStateDistanceFnPtr dist_fn);
                ~SliceCache();
                SlicePtr getNewSlice(ompl::planning::essentials::MotionPtr motion);
                void cacheSlice(SlicePtr slice);

            private:
                std::stack<SlicePtr> _slices_cache;
                RobotStateDistanceFnPtr _dist_fn;
            };

            class DebugDrawer : public std::enable_shared_from_this<DebugDrawer> {
            public:
                DebugDrawer(sim_env::WorldViewerPtr world, unsigned int robot_id, const std::vector<unsigned int>& target_indices);
                DebugDrawer(sim_env::WorldViewerPtr world, SliceDrawerInterfacePtr slice_drawer, unsigned int robot_id, const std::vector<unsigned int>& target_indices);
                ~DebugDrawer();
                void setSliceDrawer(SliceDrawerInterfacePtr slice_drawer);
                void setRobotId(unsigned int robot_id);
                void setTargetIds(const std::vector<unsigned int>& target_ids);
                void addNewMotion(mps::planner::ompl::planning::essentials::MotionPtr motion);
                void clear(bool clear_slice_drawer = true);
                void drawStateTransition(const ompl::state::SimEnvObjectState* parent_state,
                    const ompl::state::SimEnvObjectState* new_state,
                    const Eigen::Vector4f& color);
                void showState(const ompl::state::SimEnvWorldState* state, const ompl::state::SimEnvWorldStateSpaceConstPtr state_space);
                void addNewSlice(mps::planner::pushing::algorithm::SliceConstPtr slice);
                SliceDrawerInterfacePtr getSliceDrawer();

            private:
                sim_env::WorldViewerPtr _world_viewer;
                SliceDrawerInterfacePtr _slice_drawer;
                std::vector<sim_env::WorldViewer::Handle> _handles;
                unsigned int _robot_id;
                std::vector<unsigned int> _target_ids;
            };

            class SliceDrawerInterface {
            public:
                virtual ~SliceDrawerInterface() = 0;
                // TODO functions needed to draw a slice
                virtual void clear() = 0;
                virtual void addSlice(mps::planner::pushing::algorithm::SliceConstPtr slice) = 0;
                void setDebugDrawer(mps::planner::pushing::algorithm::DebugDrawerPtr debug_drawer);
                void setStateSpace(mps::planner::ompl::state::SimEnvWorldStateSpacePtr state_space);

            protected:
                mps::planner::ompl::state::SimEnvWorldStateSpacePtr _state_space;
                mps::planner::pushing::algorithm::DebugDrawerWeakPtr _debug_drawer;
            };
        }
    }
}
}

#endif
