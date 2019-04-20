#include <mps/planner/pushing/oracle/QuasiStaticSE2Oracle.h>
#include <mps/planner/util/Logging.h>

namespace mps_state = mps::planner::ompl::state;
namespace mps_logging = mps::planner::util::logging;
// namespace mps_control = mps::planner::ompl::control;

using namespace mps::planner::pushing::oracle;

QuasiStaticSE2Oracle::QuasiStaticSE2Oracle(const std::vector<sim_env::ObjectPtr>& objects, unsigned int robot_id)
: _robot_id(robot_id) {

}

QuasiStaticSE2Oracle::~QuasiStaticSE2Oracle() {

}

void QuasiStaticSE2Oracle::predictAction(const mps_state::SimEnvWorldState* current_state,
                    const mps_state::SimEnvWorldState* target_state,
                    const unsigned int& obj_id, ::ompl::control::Control* control)
{
    mps_logging::logWarn("Not implemented yet", "[QuasiStaticSE2Oracle::predictAction]");

}

void QuasiStaticSE2Oracle::samplePushingState(const mps_state::SimEnvWorldState* current_state,
                    const mps_state::SimEnvWorldState* next_state,
                    const unsigned int& obj_id,
                    mps_state::SimEnvObjectState* new_robot_state)
{
    mps_logging::logWarn("Not implemented yet", "[QuasiStaticSE2Oracle::samplePushingState]");

}