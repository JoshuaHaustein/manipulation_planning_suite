#pragma once
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <memory>
#include <mps/planner/ompl/control/Interfaces.h>
#include <ompl/control/ControlSpace.h>
#include <stack>

namespace mps {
namespace planner {
    namespace ompl {
        namespace control {
            class TimedWaypoints : public PositionControl, public SemiDynamicControl {
            public:
                // TimedWaypoints(const Eigen::Affine3f& tf = Eigen::Affine3f());
                TimedWaypoints();
                TimedWaypoints(const TimedWaypoints& other);
                TimedWaypoints(const std::vector<std::pair<float, Eigen::VectorXf>>& waypoints);
                // const Eigen::Affine3f& tf = Eigen::Affine3f());
                ~TimedWaypoints();
                // pairs of <time, wp> where time is the absolute time since the start of this control at which
                // the robot should be at waypoint wp. Does not perform any sanity checks
                void setWaypoints(const std::vector<std::pair<float, Eigen::VectorXf>>& waypoints);
                void addWaypoint(float time, const Eigen::VectorXf& wp);
                void reset(); // set to null control
                bool operator==(const TimedWaypoints& other) const;
                TimedWaypoints& operator=(const TimedWaypoints& other);
                void print(std::ostream& out) const;

                //PositionControl
                Eigen::VectorXf getPosition(float t) const override;
                void getPosition(float t, Eigen::VectorXf& pos) const override;
                float getDuration() const override;
                // SemiDynamicControl
                float getPreRestDuration() const override;
                float getRestDuration() const override;
                void addRestDuration(float dt) override;
                void setRestDuration(float dt) override;

            private:
                // const Eigen::Affine3f _tf;
                float _resting_time;
                // TODO if we want to use tf, need to use Eigen::Vector3f
                std::vector<std::pair<float, Eigen::VectorXf>> _waypoints;
            };

            class TimedWaypointsControlSpace : public ::ompl::control::ControlSpace,
                                               public SerializableControlSpace,
                                               public std::enable_shared_from_this<TimedWaypointsControlSpace> {

            public:
                /**
                     * Creates a new timed waypoints control space.
                     */
                TimedWaypointsControlSpace(const ::ompl::base::StateSpacePtr& stateSpace);
                ~TimedWaypointsControlSpace() override;

                /** ControlSpace */
                /**
                     * NOT SUPPORTED. Throws a std::logic_error exception.
                     */
                unsigned int getDimension() const override;
                ::ompl::control::Control* allocControl() const override;
                void freeControl(::ompl::control::Control* control) const override;
                void copyControl(::ompl::control::Control* control,
                    const ::ompl::control::Control* source) const override;
                bool equalControls(const ::ompl::control::Control* control_1,
                    const ::ompl::control::Control* control_2) const override;
                void nullControl(::ompl::control::Control* control) const override;
                void printControl(const ::ompl::control::Control* control, std::ostream& out) const override;
                void setup() override;
                bool isCompound() const override;
                void serializeSpaceInformation(std::ostream& ostream) const override;
                bool deserializeSpaceInformation(std::istream& istream) override;

                /**
                 * TODO: None of the functions below is implemented. All throw logic_error when called.
                 */
                ::ompl::control::ControlSamplerPtr allocDefaultControlSampler() const override;
                void printSettings(std::ostream& out) const override;
                unsigned int getNumParameters() const;
                unsigned int getSerializationLength() const override;
                void serialize(void* serialization, const ::ompl::control::Control* ctrl) const override;
                void deserialize(::ompl::control::Control* ctrl, const void* serialization) const override;
                double* getValueAddressAtIndex(::ompl::control::Control* control, unsigned int index) const override;

            private:
                mutable std::stack<TimedWaypoints*> _control_cache;
            };
        }
    }
}
}
