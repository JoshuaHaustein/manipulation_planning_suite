#pragma once
#include <mps/planner/ompl/control/Interfaces.h>

namespace mps {
namespace planner {
    namespace ompl {
        namespace control {
            class TimedWaypoints : public PositionControl, public SemiDynamicControl {
                public:
                // TimedWaypoints(const Eigen::Affine3f& tf = Eigen::Affine3f());
                TimedWaypoints(const std::vector<std::pair<float, Eigen::VectorXf> >& waypoints,
                               const Eigen::Affine3f& tf = Eigen::Affine3f() );
                ~TimedWaypoints();

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
                const Eigen::Affine3f _tf;
                float _resting_time;
                // TODO if we want to use tf, need to use Eigen::Vector3f
                const std::vector<std::pair<float, Eigen::VectorXf> > _waypoints;

            };

            class TimedWaypointsControlSpace : public ::ompl::control::ControlSpace,
                                             public std::enable_shared_from_this<TimedWaypointsControlSpace> {

            public:
                /**
                     * Creates a new timed waypoints control space.
                     */
                TimedWaypointsControlSpace();
                ~TimedWaypointsControlSpace() override;

                /** ControlSpace */
                unsigned int getDimension() const override;
                ::ompl::control::Control* allocControl() const override;
                void freeControl(::ompl::control::Control* control) const override;
                void copyControl(::ompl::control::Control* control,
                    const ::ompl::control::Control* source) const override;
                bool equalControls(const ::ompl::control::Control* control_1,
                    const ::ompl::control::Control* control_2) const override;
                void nullControl(::ompl::control::Control* control) const override;
                ::ompl::control::ControlSamplerPtr allocDefaultControlSampler() const override;
                void printControl(const ::ompl::control::Control* control, std::ostream& out) const override;
                void printSettings(std::ostream& out) const override;
                void setup() override;
                unsigned int getNumParameters() const; // TODO this function is probably doing the same as getSerializationLength() is intended for
                // TODO not implemented
                unsigned int getSerializationLength() const override;
                // TODO not implemented
                void serialize(void* serialization, const ::ompl::control::Control* ctrl) const override;
                // TODO not implemented
                void deserialize(::ompl::control::Control* ctrl, const void* serialization) const override;
                bool isCompound() const override;
                /**
                     * NOT SUPPORTED. Throws a std::logic_error exception.
                     */
                double* getValueAddressAtIndex(::ompl::control::Control* control, unsigned int index) const override;

            private:
            };

        }
    }
}
}

