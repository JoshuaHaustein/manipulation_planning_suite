#include <mps/planner/pushing/oracle/QuasiStaticSE2Oracle.h>
#include <mps/planner/util/Logging.h>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/index/rtree.hpp>

namespace mps_state = mps::planner::ompl::state;
namespace mps_logging = mps::planner::util::logging;
namespace bg = boost::geometry;
// namespace mps_control = mps::planner::ompl::control;

using namespace mps::planner::pushing::oracle;

QuasiStaticSE2Oracle::Parameters::Parameters(): eps_min(0.005f), eps_dist(0.005f) {
}

QuasiStaticSE2Oracle::QuasiStaticSE2Oracle(const std::vector<sim_env::ObjectPtr>& objects, unsigned int robot_id)
    : _robot_id(robot_id)
    , _objects(objects)
    , _random_gen(mps::planner::util::random::getDefaultRandomGenerator())
{
    computeRobotPushingEdges();
    computeObjectPushingEdges();
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
    static const std::string log_prefix("[QuasiStaticSE2Oracle::samplePushingState]");
    mps_logging::logDebug("samplePushingState", log_prefix);
    assert(obj_id < _contact_pairs.size());
    // compute sampling weights for each pair base on pushing direction
    std::vector<float> sampling_weights;
    float normalizer = computeSamplingWeights(current_state, next_state, obj_id, sampling_weights);
    auto& pushing_edge_pairs = _contact_pairs.at(obj_id);
    // sample pair
    float min_translation = 0.0f, max_translation = 0.0f;
    RobotPushingEdgePtr robot_edge;
    ObjectPushingEdgePtr obj_edge;
    float die = _random_gen->uniform01();
    float acc = 0.0f;
    for (unsigned int pair_id = 0; pair_id < pushing_edge_pairs.size(); ++pair_id) {
        acc += sampling_weights.at(pair_id) / normalizer;
        if (die <= acc || pair_id == pushing_edge_pairs.size() - 1) {
            robot_edge = pushing_edge_pairs.at(pair_id).robot_edge;
            obj_edge = pushing_edge_pairs.at(pair_id).object_edge;
            min_translation = pushing_edge_pairs.at(pair_id).min_translation;
            max_translation = pushing_edge_pairs.at(pair_id).max_translation;
            break;
        }
    }
    // place the robot such that its pushing edge faces the object's pushing edge
    float rel_trans = _random_gen->uniformReal(min_translation, max_translation);
    Eigen::Vector2f pusher_pos = obj_edge->pcom + obj_edge->normal * _params.eps_dist + rel_trans * obj_edge->dir;
    Eigen::Affine2f oTp = Eigen::Translation2f(pusher_pos) * Eigen::Rotation2D(obj_edge->pushing_angle);
    current_state->getObjectState(obj_id)->getConfiguration(_eigen_config);
    Eigen::Affine2f wTo = Eigen::Translation2f(_eigen_config.head(2)) * Eigen::Rotation2Df(_eigen_config[2]);
    Eigen::Affine2f rTp = Eigen::Translation2f(robot_edge->center) * Eigen::Rotation2Df(std::atan2(robot_edge->normal[1], robot_edge->normal[0]));
    Eigen::Affine2f pTr = rTp.inverse();
    Eigen::Affine2f wTr = wTo * oTp * pTr;
    _eigen_config.head(2) = wTr.translation();
    _eigen_config[2] = std::atan2(wTr.rotation()(1, 0), wTr.rotation()(0, 0));
    new_robot_state->setConfiguration(_eigen_config);
    auto logger = mps_logging::getLogger();
    logger->logDebug(boost::format("Sampled pushing state %1%, %2%, %3%") % _eigen_config[0] % _eigen_config[1] % _eigen_config[2], log_prefix);
    _eigen_config.setZero();
    new_robot_state->setVelocity(_eigen_config);
}

void QuasiStaticSE2Oracle::computeRobotPushingEdges()
{
    if (!_robot_edges.empty())
        return;
    auto robot = std::dynamic_pointer_cast<sim_env::Robot>(_objects.at(_robot_id));
    assert(robot);
    std::vector<sim_env::LinkPtr> links;
    robot->getLinks(links);
    for (auto link : links) {
        // TODO in case we have multiple links we would need to filter edges that only face other links
        std::vector<sim_env::Geometry> geometries(link->getGeometries());
        for (sim_env::Geometry& geom : geometries) {
            for (unsigned int vid = 1; vid < geom.vertices.size(); ++vid) {
                RobotPushingEdgePtr edge = std::make_shared<RobotPushingEdge>();
                edge->from = geom.vertices.at(vid - 1).head(2);
                edge->to = geom.vertices.at(vid).head(2);
                edge->center = (edge->from + edge->to) / 2.0f;
                edge->dir = edge->to - edge->from;
                edge->edge_length = edge->dir.norm();
                // check whether this edge is long enough
                if (edge->edge_length < 2.0 * _params.eps_min) {
                    continue;
                }
                edge->dir /= edge->edge_length;
                edge->normal[0] = edge->dir[1];
                edge->normal[1] = -edge->dir[0];
                _robot_edges.push_back(edge);
            }
        }
    }
}

typedef std::tuple<Eigen::Vector2f, Eigen::Vector2f, bool> Edge;
typedef bg::model::d2::point_xy<float> BoostPoint;
typedef bg::model::polygon<BoostPoint, false> BoostPolygon;
typedef bg::model::segment<BoostPoint> BoostSegment;

void extractEdges(BoostPolygon& geom, std::vector<Edge>& edges) {
    // first add all edges of the original geometry and also store it in an rtree
    bg::index::rtree<BoostSegment, bg::index::linear<16>> tree;
    {
        auto& outer = geom.outer();
        for (unsigned int vid = 1; vid < outer.size(); ++ vid) {
            tree.insert(BoostSegment(outer[vid-1], outer[vid]));
            edges.emplace_back(std::make_tuple(Eigen::Vector2f(outer[vid - 1].get<0>(), outer[vid-1].get<1>()),
                                                Eigen::Vector2f(outer[vid].get<0>(), outer[vid].get<1>()), false));
        }
    }
    // now run over the convex hull of the object and add the edges that aren't in rtree yet
    {
        BoostPolygon hull;
        bg::convex_hull(geom, hull);
        auto& outer = hull.outer();
        for (unsigned int vid = 1; vid < outer.size(); ++vid) {
            auto seg = BoostSegment(outer[vid - 1], outer[vid]);
            std::vector<BoostSegment> intersecting_edges;
            // tree.query(bg::index::nearest(seg, 1), std::back_inserter(nearest_edge));
            tree.query(bg::index::intersects(seg), std::back_inserter(intersecting_edges));
            // float edge_distance = bg::comparable_distance(seg, nearest_edge.back());
            // float edge_distance2 = bg::distance(seg, nearest_edge.back());
            bool normal_edge = false;
            for (auto& real_edge : intersecting_edges) {
                BoostSegment tseg(real_edge.first, real_edge.second);
                if (bg::equals(seg, tseg)) {
                    normal_edge = true;
                    break;
                }
            }
            // bool normal_edge = edge_distance == 0.0;
            if (!normal_edge) {
                edges.emplace_back(std::make_tuple(Eigen::Vector2f(outer[vid - 1].get<0>(), outer[vid-1].get<1>()),
                                                Eigen::Vector2f(outer[vid].get<0>(), outer[vid].get<1>()), true));
            }
        }
    }
}

void QuasiStaticSE2Oracle::computeObjectPushingEdges() 
{
    if (!_obj_edges.empty()) return;
    _obj_edges.resize(_objects.size());
    _contact_pairs.resize(_objects.size());
    // run over all objects
    for (unsigned int oid = 0; oid < _objects.size(); ++oid) {
        if (oid == _robot_id) { // skip robot
            continue;
        }
        auto obj = _objects.at(oid);
        // compute pushing edges for this object
        std::vector<sim_env::LinkPtr> links;
        obj->getLinks(links);
        // can only work with rigid bodies
        assert(links.size() == 1);
        Eigen::Vector3f com;
        sim_env::LinkPtr link = links.at(0);
        link->getLocalCenterOfMass(com);
        // get object geometry and compute convex hull
        std::vector<sim_env::Geometry> geometries(link->getGeometries());
        // TODO support multiple disjoint geometries?
        assert(geometries.size() == 1);
        // for this translate geometry into boost polygons
        BoostPolygon bp;
        for (auto& vertex : geometries.at(0).vertices) {
            bg::append(bp, BoostPoint(vertex[0], vertex[1]));
        }
        std::vector<Edge> edges;
        extractEdges(bp, edges);
        // now run over all edges and extract pushing edges
        for (auto& edge : edges) {
            ObjectPushingEdgePtr pedge = std::make_shared<ObjectPushingEdge>();
            pedge->from = std::get<0>(edge);
            pedge->to = std::get<1>(edge);
            pedge->dir = pedge->to - pedge->from;
            pedge->edge_length = pedge->dir.norm();
            // check whether this edge is long enough
            if (pedge->edge_length < 2.0f * _params.eps_min) {
                continue;
            }
            pedge->dir /= pedge->edge_length;
            pedge->normal[0] = pedge->dir[1];
            pedge->normal[1] = -pedge->dir[0];
            pedge->pushing_angle = std::atan2(-pedge->normal[1], -pedge->normal[0]);
            // test whether this edge is suitable for stable pushing
            Eigen::Vector2f rel_com = com.head(2) - pedge->from;
            rel_com = rel_com - rel_com.dot(pedge->normal) * pedge->normal;
            float sdist = rel_com.dot(pedge->dir);
            pedge->pcom = rel_com + pedge->from;
            // check whether the projected center of mass falls onto this edge
            if (sdist < _params.eps_min or sdist > pedge->edge_length - _params.eps_min) {
                continue;
            }
            _obj_edges.at(oid).push_back(pedge);
            // compute what robot edge can be used to push this edge
            float min_edge_length = 2.0f * _params.eps_min;
            if (std::get<2>(edge)) { // object edge is from convex hull, i.e. only end points are actually part of the object
                min_edge_length = pedge->edge_length;
            }
            for (auto& robot_edge : _robot_edges) {
                if (robot_edge->edge_length > min_edge_length) {
                    PushingEdgePair pepair;
                    pepair.robot_edge = robot_edge;
                    pepair.object_edge =pedge;
                    // TODO do collision check to compue min and max translation
                    pepair.max_translation = 0.0f;
                    pepair.min_translation = 0.0f;
                    _contact_pairs.at(oid).push_back(pepair);
                }
            }
        }
        if (_contact_pairs.at(oid).empty()) {
            throw std::runtime_error("Failed to compute any pushing edge pairs for object " + _objects.at(oid)->getName());
        }
    }
}

float QuasiStaticSE2Oracle::computeSamplingWeights(const ompl::state::SimEnvWorldState* current_state, 
                                        const ompl::state::SimEnvWorldState* next_state, unsigned int obj_id,
                                        std::vector<float>& sampling_weights) const 
{
    // compute pushing dir
    current_state->getObjectState(obj_id)->getConfiguration(_eigen_config);
    next_state->getObjectState(obj_id)->getConfiguration(_eigen_config2);
    Eigen::Vector2f dir = _eigen_config2.head(2) - _eigen_config.head(2);
    dir.normalize();
    Eigen::Rotation2D rot(_eigen_config[2]);
    // rate all pushing edge pairs
    float normalizer = 0.0f;
    auto& contact_pairs = _contact_pairs.at(obj_id);
    for (unsigned int pid = 0; pid < _contact_pairs.at(obj_id).size(); ++pid) {
        Eigen::Vector2f edge_normal = rot * contact_pairs.at(pid).object_edge->normal;
        sampling_weights.emplace_back(std::exp(-(1.0f + dir.dot(edge_normal))));
        normalizer += sampling_weights.back();
    }
    return normalizer;
}
