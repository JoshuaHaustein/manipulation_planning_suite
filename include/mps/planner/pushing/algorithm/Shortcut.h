//
// Created by joshua on 8/14/17.
//

#ifndef MPS_ALGORITHM_SHORTCUT_H
#define MPS_ALGORITHM_SHORTCUT_H

// OMPL includes
#include <ompl/base/Path.h>
#include <ompl/base/State.h>
#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/control/SpaceInformation.h>
// MPS includes
#include <mps/planner/ompl/control/SimEnvStatePropagator.h>
#include <mps/planner/pushing/algorithm/RearrangementPlanner.h>
#include <mps/planner/util/Random.h>
#include <mps/planner/util/Time.h>
// #include <mps/planner/ompl/control/NaiveControlSampler.h>
#include <mps/planner/pushing/oracle/OracleControlSampler.h>
// stl
#include <stack>
// boost
#include <boost/functional/hash.hpp>

namespace mps {
namespace planner {
    namespace pushing {
        namespace algorithm {
            //////////////////////////////////////////////////////////////////////////////////////
            /////////////////////////////// Shortcutting algorithms //////////////////////////////
            //////////////////////////////////////////////////////////////////////////////////////
            class Shortcutter {
                /***
                     * A shortcutter takes a path found by a  and shortcuts it, i.e. it tries 
                     * to remove unneccessary actions from the solutions.
                     */
            public:
                struct ShortcutQuery {
                    ompl::state::goal::ObjectsRelocationGoalPtr goal_region; // goal region
                    ompl::planning::essentials::CostFunctionPtr cost_function; // objective for shortcutting
                    std::string robot_name;
                    std::function<bool()> stopping_condition; // optionally a customized stopping condition
                    float cost_before_shortcut;
                    float cost_after_shortcut;
                    ShortcutQuery(ompl::state::goal::ObjectsRelocationGoalPtr goal_region_in,
                        ompl::planning::essentials::CostFunctionPtr cost_function_in,
                        std::string robot_name_in)
                    {
                        stopping_condition = []() { return false; };
                        goal_region = goal_region_in;
                        cost_function = cost_function_in;
                        robot_name = robot_name_in;
                    }
                };
                Shortcutter(::ompl::control::SpaceInformationPtr si);
                virtual ~Shortcutter() = 0;

                void setDebugDrawer(mps::planner::pushing::algorithm::DebugDrawerPtr debug_drawer);

                /**
                         *  Shortcut the given path.
                         * @param path - a path created by a rearrangement RRT. If the path is invalid,
                         *               the path remains unchanged, else the path is shortcut.
                         * @param pq - shortcut query
                         * @max_time - maximal duration this function is allowed to run (in seconds)
                         */
                virtual void shortcut(mps::planner::ompl::planning::essentials::PathPtr path,
                    ShortcutQuery& pq,
                    float max_time)
                    = 0;
                virtual std::string getName() const = 0;

            protected:
                ::ompl::control::SpaceInformationPtr _si;
                mps::planner::ompl::control::SimEnvStatePropagatorPtr _state_propagator;
                mps::planner::util::time::Timer _timer;
                MotionCache<> _motion_cache;
                // Methods
                mps::planner::ompl::planning::essentials::PathPtr getNewPath();
                // caches the given path, if clear_id >= 0, the motions with id >= clear_id are also cached
                void cachePath(mps::planner::ompl::planning::essentials::PathPtr ptr, int clear_id = -1);
                /***
                         *  Forward propagates the actions stored in new_motions starting from the last state stored in path.
                         *  All successive states are added to path.  Aborts execution if any intermediate state is invalid or a goal
                         *  Returns pair <b_success, b_goal>, where b_success is true iff all propagations were valid
                         *      and b_goal is true iff path->last() is a goal.
                         */
                std::pair<bool, bool> forwardPropagatePath(mps::planner::ompl::planning::essentials::PathPtr path,
                    std::vector<mps::planner::ompl::planning::essentials::MotionPtr>& new_motions,
                    ShortcutQuery& sq);
                std::pair<bool, bool> forwardPropagatePath(mps::planner::ompl::planning::essentials::PathPtr path,
                    const std::vector<const ::ompl::control::Control*>& controls,
                    ShortcutQuery& sq);
                /***
                         *  Forward propagates the actions stored in old_path starting from the state
                         *  stored with id old_path_continuation.
                         *  All successive states are added to path.  
                         *  Aborts execution if any intermediate state is invalid or a goal
                         *  Returns pair <b_success, b_goal>, where b_success is true iff all propagations were valid
                         *      and b_goal is true iff path->last() is a goal.
                         */
                std::pair<bool, bool> forwardPropagatePath(mps::planner::ompl::planning::essentials::PathPtr path,
                    mps::planner::ompl::planning::essentials::PathPtr old_path,
                    unsigned int old_path_continuation,
                    ShortcutQuery& sq);
                /***
                         * Forward propagates the action stored in new_wp and from there on all actions
                         * in old_path starting at index old_path_continuation. The resulting states
                         * are added as new motions to path. The propagation starts from the last state stored
                         * in path.
                         *  Returns pair <b_success, b_goal>, where b_success is true iff all propagations were valid
                         *      and b_goal is true iff path->last() is a goal.
                         */
                std::pair<bool, bool> forwardPropagatePath(mps::planner::ompl::planning::essentials::PathPtr path,
                    std::vector<mps::planner::ompl::planning::essentials::MotionPtr>& new_motions,
                    mps::planner::ompl::planning::essentials::PathPtr old_path,
                    unsigned int old_path_continuation,
                    ShortcutQuery& sq);
                void showState(::ompl::base::State* state, const std::string& msg);
                mps::planner::pushing::algorithm::DebugDrawerPtr _debug_drawer;

            private:
                std::stack<mps::planner::ompl::planning::essentials::PathPtr> _path_cache;
            };
            typedef std::shared_ptr<Shortcutter> ShortcutterPtr;
            typedef std::shared_ptr<const Shortcutter> ShortcutterConstPtr;
            typedef std::weak_ptr<Shortcutter> ShortcutterWeakPtr;
            typedef std::weak_ptr<const Shortcutter> ShortcutterWeakConstPtr;

            class NaiveShortcutter : public Shortcutter, public std::enable_shared_from_this<NaiveShortcutter> {
                /**
                     * A NaiveShortCutter attempts to shortcut a path by steering the robot between two randomly 
                     * sampled states (x_i, x_j), j > i + 1, and checking whether the subsequent actions from 
                     * state x_j still lead to success.
                     */
            public:
                NaiveShortcutter(::ompl::control::SpaceInformationPtr si,
                    mps::planner::pushing::oracle::RobotOraclePtr robot_oracle);
                virtual ~NaiveShortcutter();
                void shortcut(mps::planner::ompl::planning::essentials::PathPtr path,
                    ShortcutQuery& pq,
                    float max_time) override;
                std::string getName() const override;

            protected:
                void computeRobotActions(std::vector<mps::planner::ompl::planning::essentials::MotionPtr>& motions,
                    ::ompl::base::State* start_state,
                    ::ompl::base::State* end_state,
                    unsigned int robot_id);
                // creates all pairs of waypoints
                void createAllPairs(std::vector<std::pair<unsigned int, unsigned int>>& all_pairs, unsigned int n) const;

            private:
                mps::planner::pushing::oracle::RobotOraclePtr _robot_oracle;
            };
            typedef std::shared_ptr<NaiveShortcutter> NaiveShortCutterPtr;
            typedef std::shared_ptr<const NaiveShortcutter> NaiveShortCutterConstPtr;
            typedef std::weak_ptr<NaiveShortcutter> NaiveShortCutterWeakPtr;
            typedef std::weak_ptr<const NaiveShortcutter> NaiveShortCutterWeakConstPtr;

            class LocalShortcutter : public Shortcutter {
            public:
                LocalShortcutter(::ompl::control::SpaceInformationPtr si,
                    mps::planner::pushing::oracle::RobotOraclePtr robot_oracle,
                    const std::string& robot_name);
                virtual ~LocalShortcutter();
                void shortcut(mps::planner::ompl::planning::essentials::PathPtr path,
                    ShortcutQuery& pq,
                    float max_time) override;
                std::string getName() const override;

            protected:
                /** Computes a shortcut from the state in prefix_path->last() to end_state and extends prefix_path
                         * with the respective motions.
                         * @return boolean pair (extension_success, path->last() is a goal)
                        */
                virtual std::pair<bool, bool> computeShortcut(mps::planner::ompl::planning::essentials::PathPtr prefix_path,
                    ::ompl::base::State* end_state, ShortcutQuery& pq);
                void computeRobotActions(std::vector<mps::planner::ompl::planning::essentials::MotionPtr>& motions,
                    ::ompl::base::State* start_state,
                    ::ompl::base::State* end_state);
                unsigned int _robot_id;

            private:
                mps::planner::pushing::oracle::RobotOraclePtr _robot_oracle;
            };

            class LocalOracleShortcutter : public LocalShortcutter {
            public:
                LocalOracleShortcutter(::ompl::control::SpaceInformationPtr si,
                    mps::planner::pushing::oracle::RobotOraclePtr robot_oracle,
                    mps::planner::pushing::oracle::PushingOraclePtr pushing_oracle,
                    const std::string& robot_name);
                virtual ~LocalOracleShortcutter();
                std::string getName() const override;

            protected:
                // computes which object the shortcutting between start_state and end_state should focus on.
                // both states are assumend to be SimEnvWorldStates
                unsigned int selectObject(::ompl::base::State* start_state, ::ompl::base::State* end_state);
                std::pair<bool, bool> computeShortcut(mps::planner::ompl::planning::essentials::PathPtr prefix_path,
                    ::ompl::base::State* end_state, ShortcutQuery& pq) override;

            private:
                mps::planner::pushing::oracle::OracleControlSamplerPtr _oracle_sampler;
            };

            typedef std::shared_ptr<LocalOracleShortcutter> LocalOracleShortcutterPtr;
            typedef std::shared_ptr<const LocalOracleShortcutter> LocalOracleShortcutterConstPtr;
            typedef std::weak_ptr<LocalOracleShortcutter> LocalOracleShortcutterWeakPtr;
            typedef std::weak_ptr<const LocalOracleShortcutter> LocalOracleShortcutterWeakConstPtr;

            class OracleShortcutter : public Shortcutter, public std::enable_shared_from_this<OracleShortcutter> {
                /**
                     * A OracleShortcutter attempts to shortcut a path by using the oracle to move between two randomly 
                     * sampled states (x_i, x_j), j > i + 1, and checking whether the subsequent actions from 
                     * state x_j still lead to success. In contrast to the NaiveShortCutter, this shortcutter attempts
                     * to push an object towards its respective state in x_j before steering the robot to its state.
                     */
            public:
                OracleShortcutter(::ompl::control::SpaceInformationPtr si,
                    mps::planner::pushing::oracle::RobotOraclePtr robot_oracle,
                    mps::planner::pushing::oracle::PushingOraclePtr pushing_oracle,
                    const std::string& robot_name);
                virtual ~OracleShortcutter();
                void shortcut(mps::planner::ompl::planning::essentials::PathPtr path,
                    ShortcutQuery& pq,
                    float max_time) override;
                std::string getName() const override;

            protected:
                typedef std::deque<std::pair<unsigned int, unsigned int>> PairQueue;
                typedef std::vector<unsigned int> ObjectIds;
                typedef std::unordered_map<std::pair<unsigned int, unsigned int>, ObjectIds, boost::hash<std::pair<unsigned int, unsigned int>>> PairMap;

            private:
                mps::planner::pushing::oracle::OracleControlSamplerPtr _oracle_sampler;
                // Selects which object to push for shortcutting and updates pair data structures
                unsigned int selectObject(mps::planner::ompl::planning::essentials::PathPtr current_path,
                    std::pair<unsigned int, unsigned int>& current_pair,
                    PairMap& pair_to_objects, PairQueue& pair_queue,
                    unsigned int robot_id) const;
                // computes the actual shortcut and extends new_path
                bool computeShortcut(mps::planner::ompl::planning::essentials::PathPtr new_path,
                    mps::planner::ompl::planning::essentials::PathPtr current_path,
                    unsigned int target_id, unsigned int robot_id, unsigned int object_id,
                    ShortcutQuery& sq);
                void fillPairQueue(PairQueue& all_pairs, unsigned int n) const;
                bool extendPath(mps::planner::ompl::planning::essentials::PathPtr path,
                    const std::vector<const ::ompl::control::Control*>& controls,
                    bool& goal_reached,
                    ShortcutQuery& sq);
            };
            typedef std::shared_ptr<OracleShortcutter> OracleShortcutterPtr;
            typedef std::shared_ptr<const OracleShortcutter> OracleShortcutterConstPtr;
            typedef std::weak_ptr<OracleShortcutter> OracleShortcutterWeakPtr;
            typedef std::weak_ptr<const OracleShortcutter> OracleShortcutterWeakConstPtr;
        }
    }
}
}

#endif