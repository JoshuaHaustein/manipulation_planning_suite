#include <boost/geometry.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/index/rtree.hpp>
#include <mps/planner/pushing/oracle/QuasiStaticSE2Oracle.h>
#include <mps/planner/util/Logging.h>

namespace mps_state = mps::planner::ompl::state;
namespace mps_logging = mps::planner::util::logging;
namespace bg = boost::geometry;
// namespace mps_control = mps::planner::ompl::control;

using namespace mps::planner::pushing::oracle;

QuasiStaticSE2Oracle::Parameters::Parameters()
    : eps_min(0.005f)
    , eps_dist(0.005f)
    , col_sample_step(0.0025f)
{
}

QuasiStaticSE2Oracle::QuasiStaticSE2Oracle(const std::vector<sim_env::ObjectPtr>& objects, unsigned int robot_id)
    : _robot_id(robot_id)
    , _objects(objects)
    , _random_gen(mps::planner::util::random::getDefaultRandomGenerator())
{
    computeRobotPushingEdges();
    computeObjectPushingEdges();
    _tmp_edge_counter = 0;
    _min_max_toggle = false;
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
    unsigned int pair_id = 0;
    // float die = _random_gen->uniform01();
    // float acc = 0.0f;
    // for (unsigned int id = 0; id < pushing_edge_pairs.size(); ++id) {
    //     acc += sampling_weights.at(pair_id) / normalizer;
    //     if (die <= acc || pair_id == pushing_edge_pairs.size() - 1) {
    //         pair_id = id;
    //         break;
    //     }
    // }
    // TODO remove
    _tmp_edge_counter = _tmp_edge_counter % pushing_edge_pairs.size();
    pair_id = _tmp_edge_counter;
    _tmp_edge_counter += _min_max_toggle;
    _min_max_toggle = !_min_max_toggle;
    // TODO until here
    // place the robot such that its pushing edge faces the object's pushing edge
    auto& edge_pair = pushing_edge_pairs.at(pair_id);
    // float rel_trans = _random_gen->uniformReal(edge_pair.min_translation, edge_pair.max_translation);
    float rel_trans = _min_max_toggle ? edge_pair.max_translation : edge_pair.min_translation;
    current_state->getObjectState(obj_id)->getConfiguration(_eigen_config);
    computeRobotState(_eigen_config2, edge_pair, _eigen_config, rel_trans);
    new_robot_state->setConfiguration(_eigen_config2);
    auto logger = mps_logging::getLogger();
    logger->logDebug(boost::format("Sampled pushing state %1%, %2%, %3%") % _eigen_config2[0] % _eigen_config2[1] % _eigen_config2[2], log_prefix);
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

void extractEdges(BoostPolygon& geom, std::vector<Edge>& edges)
{
    // first add all edges of the original geometry and also store it in an rtree
    bg::index::rtree<BoostSegment, bg::index::linear<16>> tree;
    {
        auto& outer = geom.outer();
        for (unsigned int vid = 1; vid < outer.size(); ++vid) {
            tree.insert(BoostSegment(outer[vid - 1], outer[vid]));
            edges.emplace_back(std::make_tuple(Eigen::Vector2f(outer[vid - 1].get<0>(), outer[vid - 1].get<1>()),
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
                edges.emplace_back(std::make_tuple(Eigen::Vector2f(outer[vid - 1].get<0>(), outer[vid - 1].get<1>()),
                    Eigen::Vector2f(outer[vid].get<0>(), outer[vid].get<1>()), true));
            }
        }
    }
}

void QuasiStaticSE2Oracle::computeObjectPushingEdges()
{
    if (!_obj_edges.empty())
        return;
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
                    pepair.object_edge = pedge;
                    pepair.max_translation = 0.0f;
                    pepair.min_translation = 0.0f;
                    if (std::get<2>(edge)) {
                        pepair.min_translation = (pedge->to - pedge->pcom).norm() - robot_edge->edge_length / 2.0f;
                        pepair.max_translation = robot_edge->edge_length / 2.0f - (pedge->from - pedge->pcom).norm();
                    } else {
                        pepair.min_translation = -std::max(robot_edge->edge_length / 2.0f - _params.eps_min, 0.0f);
                        pepair.max_translation = std::max(robot_edge->edge_length / 2.0f - _params.eps_min, 0.0f);
                    }
                    // bool valid = true;
                    bool valid = computeCollisionFreeRange(pepair, oid);
                    if (valid)
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

void QuasiStaticSE2Oracle::computeRobotState(Eigen::VectorXf& rob_state, const QuasiStaticSE2Oracle::PushingEdgePair& pair,
    const Eigen::VectorXf& obj_state, float translation) const
{
    Eigen::Vector2f pusher_pos = pair.object_edge->pcom + pair.object_edge->normal * _params.eps_dist + translation * pair.object_edge->dir;
    Eigen::Affine2f oTp = Eigen::Translation2f(pusher_pos) * Eigen::Rotation2D(pair.object_edge->pushing_angle);
    Eigen::Affine2f wTo = Eigen::Translation2f(obj_state.head(2)) * Eigen::Rotation2Df(obj_state[2]);
    Eigen::Affine2f rTp = Eigen::Translation2f(pair.robot_edge->center) * Eigen::Rotation2Df(std::atan2(pair.robot_edge->normal[1], pair.robot_edge->normal[0]));
    Eigen::Affine2f pTr = rTp.inverse();
    Eigen::Affine2f wTr = wTo * oTp * pTr;
    rob_state.head(2) = wTr.translation();
    rob_state[2] = std::atan2(wTr.rotation()(1, 0), wTr.rotation()(0, 0));
}

bool QuasiStaticSE2Oracle::computeCollisionFreeRange(QuasiStaticSE2Oracle::PushingEdgePair& pair, unsigned int oid)
{
    assert(_objects.at(oid)->getNumActiveDOFs() == 3); // this only works for planar rigid bodies
    auto robot = _objects.at(_robot_id);
    _eigen_config.resize(3);
    assert(robot->getNumActiveDOFs() == 3); // this only works for planar robots
    auto obj_state(_objects.at(oid)->getDOFPositions());
    // init variables for algorithm
    bool col_free_exists = false; // whether there is any collision-free state
    auto best_interval = std::make_pair(pair.min_translation, pair.min_translation); // best interval of translations
    float best_interval_length = 0.0f; // length of that interval
    auto current_interval = std::make_pair(pair.min_translation, pair.min_translation); // current interval under investigation
    bool current_interval_valid = false; // whether we currently have a valid interval growing

    // sample range [pair.min_translation, pair.max_translation] to find the largest collision-free subinterval
    float current_t = pair.min_translation;
    while (current_t <= pair.max_translation) {
        // check collision for current_t
        computeRobotState(_eigen_config, pair, obj_state, current_t);
        robot->setDOFPositions(_eigen_config);
        if (robot->checkCollision(_objects.at(oid))) { // does it collide?
            // if the current interval is valid, close it
            if (current_interval_valid) {
                float interval_length = current_interval.second - current_interval.first;
                // save it if it's longer than the best one so far
                if (interval_length >= best_interval_length) {
                    best_interval_length = interval_length;
                    best_interval = current_interval;
                }
                current_interval_valid = false;
            }
        } else {
            col_free_exists = true;
            if (current_interval_valid) {
                // grow interval
                current_interval.second = current_t;
            } else { // start new interval
                current_interval.first = current_t;
                current_interval_valid = true;
            }
        }
        current_t += _params.col_sample_step;
    }
    // close the last current_interval in case it is valid
    if (current_interval_valid) {
        float interval_length = current_interval.second - current_interval.first;
        if (interval_length >= best_interval_length) {
            best_interval_length = interval_length;
            best_interval = current_interval;
        }
    }
    // save the best interval
    pair.min_translation = best_interval.first;
    pair.max_translation = best_interval.second;
    assert(pair.min_translation <= pair.max_translation);
    return col_free_exists;
}