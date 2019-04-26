#include <mps/planner/pushing/oracle/QuasiStaticSE2Oracle.h>
#include <mps/planner/util/Logging.h>

namespace mps_state = mps::planner::ompl::state;
namespace mps_logging = mps::planner::util::logging;
// namespace mps_control = mps::planner::ompl::control;

using namespace mps::planner::pushing::oracle;

QuasiStaticSE2Oracle::QuasiStaticSE2Oracle(const std::vector<sim_env::ObjectPtr>& objects, unsigned int robot_id)
    : _robot_id(robot_id)
    , _objects(objects)
{
}

QuasiStaticSE2Oracle::~QuasiStaticSE2Oracle()
{
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

void QuasiStaticSE2Oracle::compute_robot_pushing_edges()
{
    if (!_robot_edges.empty())
        return;
    auto robot = std::dynamic_pointer_cast<sim_env::Robot>(_objects.at(_robot_id));
    assert(robot);
    Eigen::Vector3f com;
    std::vector<sim_env::LinkPtr> links;
    robot->getLinks(links);
    for (auto link : links) {
        std::vector<sim_env::Geometry> geometries(link->getGeometries());
        link->getLocalCenterOfMass(com);
        for (sim_env::Geometry& geom : geometries) {
            for (unsigned int vid = 1; vid < geom.vertices.size(); ++vid) {
                RobotPushingEdge edge;
                edge.from = geom.vertices.at(vid - 1).head(2);
                edge.to = geom.vertices.at(vid).head(2);
                edge.dir = edge.to - edge.from;
                edge.edge_length = edge.dir.norm();
                // TODO compute normal pointing outwards
                _robot_edges.push_back(edge);
            }
        }
    }
}
