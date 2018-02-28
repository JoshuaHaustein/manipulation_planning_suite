//
// Created by joshua on 9/5/17.
//

#ifndef MANIPULATION_PLANNING_SUITE_ESSENTIALS_H
#define MANIPULATION_PLANNING_SUITE_ESSENTIALS_H

#include <memory>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/Control.h>
#include <ompl/base/State.h>
#include <ompl/base/Cost.h>
#include <ompl/base/OptimizationObjective.h>

namespace mps {
    namespace planner {
        namespace ompl {
            namespace planning {
                namespace essentials {
                    class Motion;
                    typedef std::shared_ptr<Motion> MotionPtr;
                    typedef std::shared_ptr<const Motion> MotionConstPtr;

                    class Motion {
                    public:
                        Motion() = delete;
                        Motion(::ompl::control::SpaceInformationPtr si);
                        Motion(const Motion& other);
                        ~Motion();
                        Motion& operator=(const Motion& other);
                        ::ompl::base::State* getState();
                        ::ompl::base::State const* getConstState() const;
                        ::ompl::control::Control* getControl();
                        ::ompl::control::Control const* getConstControl() const;
                        MotionPtr getParent();
                        MotionConstPtr getConstParent() const;
                        void setParent(MotionPtr parent);
                    private:
                        std::weak_ptr<::ompl::control::SpaceInformation> _weak_si;
                        ::ompl::base::State* _state;
                        ::ompl::control::Control* _control;
                        MotionPtr _parent;
                    };

                    class Path : public ::ompl::base::Path {
                    public:
                        Path(::ompl::control::SpaceInformationPtr si);
                        ~Path();

                        double length() const override;
                        // DO NOT USE THIS FUNCTION - NOT IMPLEMENTED
                        ::ompl::base::Cost cost(const ::ompl::base::OptimizationObjectivePtr& oo) const override;

                        bool check() const override;
                        void print(std::ostream& out) const override;

                        /**
                         * Append a motion to this path. The motion is not copied!
                         * @param motion - motion to append to this path.
                         */
                        void append(MotionPtr motion);

                        /**
                         *  Concatenates the first n waypoints of the given path to this path.
                         *  None of the Motions are copied.
                         *  @param path - other path
                         *  @param n  - number of motions to add. if not specified the full path is concatenated
                         */
                        void concat(std::shared_ptr<Path> other, unsigned int n = std::numeric_limits<unsigned int>::max());

                        /**
                         * Resets this path and initializes this path by backtracking the path leading
                         * to motion. None of the motions are copied!
                         * @param motion - final motion of a path.
                         */
                        void initBacktrackMotion(MotionPtr motion);
                        /**
                         * Clear this path.
                         */
                        void clear();
                        unsigned int getNumMotions() const;
                        MotionPtr getMotion(unsigned int i);
                        MotionPtr last();
                        MotionConstPtr getConstMotion(unsigned int i) const;
                        /**
                         * Constructs a deep copy of this path.
                         * The deep copy copies all motions stored in this path, but not the space 
                         * information pointer
                         */
                        std::shared_ptr<Path> deepCopy() const;

                    private:
                        ::ompl::control::SpaceInformationPtr _sic;
                        std::vector<MotionPtr> _motions;
                        double _length;
                    };

                    typedef std::shared_ptr<Path> PathPtr;
                    typedef std::shared_ptr<const Path> PathConstPtr;
                    typedef std::weak_ptr<Path> PathWeakPtr;
                    typedef std::weak_ptr<const Path> PathWeakConstPtr;

                    class CostFunction {
                        public: 
                            virtual ~CostFunction() = 0;
                            /**
                             *  Returns the cost taking the action stored in second
                             *  from the state stored in first that will result in the state in second.
                             */
                            virtual double cost(MotionPtr first, MotionPtr second) = 0;

                            /**
                             * Returns the cost for a path. Default implementation accumulates
                             * cost between individual motions in path.
                             */
                            virtual double cost(PathPtr path);
                    };

                    typedef std::shared_ptr<CostFunction> CostFunctionPtr;
                    typedef std::shared_ptr<const CostFunction> CostFunctionConstPtr;
                    typedef std::weak_ptr<CostFunction> CostFunctionWeakPtr;
                    typedef std::weak_ptr<const CostFunction> CostFunctionWeakConstPtr;
                }
            }
        }
    }
}
#endif //MANIPULATION_PLANNING_SUITE_ESSENTIALS_H
