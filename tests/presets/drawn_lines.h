#ifndef HPP_GUARD_PRAXIS_TESTS_PRESETS_DRAWN_LINES_H
#define HPP_GUARD_PRAXIS_TESTS_PRESETS_DRAWN_LINES_H

#include "praxis/rigid_motion/screw_window.h"

#include <threepp/objects/Robot.hpp>

#include <threepp/scenes/Scene.hpp>

#include <threepp/core/Object3D.hpp>
#include <threepp/core/BufferGeometry.hpp>

#include <Eigen/Core>

#include <cmath>
#include <limits>
#include <string>
#include <vector>
#include <cstddef>
#include <optional>
#include <algorithm>
#include <string_view>

namespace praxis::fixture {

// Found under the object the stencil gave that name rather than by taking the first node of a type:
// a scene may carry a line under more than one object, and the order a traverse reaches them in is
// not what says which is which.
inline threepp::Object3D *first_line_under(threepp::Scene &target, std::string_view named)
{
    threepp::Object3D *found = nullptr;
    threepp::Object3D *node  = target.getObjectByName<threepp::Object3D>(std::string(named));
    if(node == nullptr)
        return nullptr;

    node->traverse(
            [&found](threepp::Object3D &at)
            {
                if(found == nullptr && at.type() == "Line")
                    found = &at;
            });

    return found;
}

// Read back in the frame the geometry was built in, which is the frame of the object carrying it.
inline std::vector<Eigen::Vector3d> line_points(threepp::Scene &target, std::string_view named)
{
    std::vector<Eigen::Vector3d> points;
    threepp::Object3D *drawn = first_line_under(target, named);
    if(drawn == nullptr)
        return points;

    const std::vector<float> &raw = drawn->geometry()->getAttribute<float>("position")->array();
    for(std::size_t at = 0; at + 2 < raw.size(); at += 3)
        points.emplace_back(raw[at], raw[at + 1], raw[at + 2]);

    return points;
}

// Where a drawn mesh stands under the object the stencil gave that name, in the frame the geometry
// was built in, and nothing where the object carries no mesh of that name. The mesh is asked for by
// name rather than by type: an object's hidden axes are meshes too, and a traverse reaches them
// first.
inline std::optional<Eigen::Vector3d> mesh_position(threepp::Scene &target, std::string_view named, std::string_view mesh)
{
    std::optional<Eigen::Vector3d> found;
    threepp::Object3D *node = target.getObjectByName<threepp::Object3D>(std::string(named));
    if(node == nullptr)
        return found;

    node->traverse(
            [&found, mesh](threepp::Object3D &at)
            {
                if(!found && at.type() == "Mesh" && at.name == mesh)
                    found = Eigen::Vector3d{at.position.x, at.position.y, at.position.z};
            });

    return found;
}

// The same points in the frame the poses are written in. The renderer's world is y-up and every
// object hangs under a root carrying that quarter turn, so a world reading is turned back through it.
inline std::vector<Eigen::Vector3d> line_in_world(threepp::Scene &target, std::string_view named)
{
    std::vector<Eigen::Vector3d> world;
    threepp::Object3D *drawn = first_line_under(target, named);
    for(const Eigen::Vector3d &at : line_points(target, named))
    {
        threepp::Vector3 put{static_cast<float>(at.x()), static_cast<float>(at.y()), static_cast<float>(at.z())};
        drawn->localToWorld(put);
        world.emplace_back(put.x, -put.z, put.y);
    }

    return world;
}

// The mesh of that name under the object the stencil gave that name, and nothing where the object
// carries none. Asked for by name rather than by type, as an object's hidden axes are meshes too.
inline threepp::Object3D *first_mesh_under(threepp::Scene &target, std::string_view named, std::string_view mesh)
{
    threepp::Object3D *found = nullptr;
    threepp::Object3D *node  = target.getObjectByName<threepp::Object3D>(std::string(named));
    if(node == nullptr)
        return nullptr;

    node->traverse(
            [&found, mesh](threepp::Object3D &at)
            {
                if(found == nullptr && at.type() == "Mesh" && at.name == mesh)
                    found = &at;
            });

    return found;
}

// The half-extent a drawn mesh occupies about its own origin on each axis, in the frame its geometry
// was built in, taken as the worst absolute vertex coordinate on that axis. A node carrying no mesh
// occupies nothing.
inline Eigen::Vector3d mesh_half_extent(threepp::Object3D *drawn)
{
    Eigen::Vector3d worst = Eigen::Vector3d::Zero();
    if(drawn == nullptr)
        return worst;

    const std::vector<float> &raw = drawn->geometry()->getAttribute<float>("position")->array();
    for(std::size_t at = 0; at + 2 < raw.size(); at += 3)
        for(Eigen::Index axis = 0; axis < 3; ++axis)
            worst[axis] = std::max(worst[axis], std::abs(static_cast<double>(raw[at + static_cast<std::size_t>(axis)])));

    return worst;
}

// Where that mesh stands in the frame the poses are written in, read through the same quarter turn
// the root carries as the lines above are.
inline std::optional<Eigen::Vector3d> mesh_in_world(threepp::Scene &target, std::string_view named, std::string_view mesh)
{
    threepp::Object3D *drawn = first_mesh_under(target, named, mesh);
    if(drawn == nullptr)
        return std::nullopt;

    threepp::Vector3 put{};
    drawn->localToWorld(put);

    return Eigen::Vector3d{put.x, -put.z, put.y};
}

// Two read lines are the same line where they carry the same points in the same order, to within
// the tolerance a point read back out of a float buffer is comparable at.
inline bool same_line(const std::vector<Eigen::Vector3d> &first, const std::vector<Eigen::Vector3d> &second)
{
    if(first.size() != second.size())
        return false;

    for(std::size_t at = 0; at < first.size(); ++at)
        if((first[at] - second[at]).norm() > 1.0e-5)
            return false;

    return true;
}

// The renderer culls a subtree at the first node whose visibility is off, so what a control did is
// read by walking upward from the drawn object rather than by asking the window what it holds.
inline bool drawn(threepp::Object3D *at)
{
    for(threepp::Object3D *node = at; node != nullptr; node = node->parent)
        if(!node->visible)
            return false;

    return at != nullptr;
}

inline threepp::Object3D *rendered_arm(threepp::Scene &target)
{
    threepp::Object3D *found = nullptr;
    target.traverseType<threepp::Robot>(
            [&found](threepp::Robot &at)
            {
                if(found == nullptr)
                    found = &at;
            });

    return found;
}

// The greatest distance two read lines stand apart at equal position along them, which is what two
// paths drawn between one pair of poses are compared by.
inline double apart(const std::vector<Eigen::Vector3d> &first, const std::vector<Eigen::Vector3d> &second)
{
    double worst = 0.0;
    for(std::size_t at = 0; at < first.size() && at < second.size(); ++at)
        worst = std::max(worst, (first[at] - second[at]).norm());

    return worst;
}

inline double from_segment(const Eigen::Vector3d &at, const Eigen::Vector3d &first, const Eigen::Vector3d &second)
{
    const Eigen::Vector3d run = second - first;
    const double along        = run.squaredNorm() > 0.0 ? std::clamp((at - first).dot(run) / run.squaredNorm(), 0.0, 1.0) : 0.0;

    return (at - (first + along * run)).norm();
}

// The nearest the drawn polyline comes to a point, taken over its segments rather than over its
// samples: a pose driven to an angle between two samples stands on the segment joining them.
inline double from_curve(const std::vector<Eigen::Vector3d> &drawn, const Eigen::Vector3d &at)
{
    double nearest = std::numeric_limits<double>::max();
    for(std::size_t step = 0; step + 1 < drawn.size(); ++step)
        nearest = std::min(nearest, from_segment(at, drawn[step], drawn[step + 1]));

    return nearest;
}

// The longest step the drawn polyline takes, which is the resolution it samples the path at.
inline double coarsest_step(const std::vector<Eigen::Vector3d> &drawn)
{
    double worst = 0.0;
    for(std::size_t step = 0; step + 1 < drawn.size(); ++step)
        worst = std::max(worst, (drawn[step + 1] - drawn[step]).norm());

    return worst;
}

// The furthest a point of the path can stand from the polyline drawn through samples of it: the
// sagitta of one sampled step on the circle the path turns about, which the drawn geometry itself
// says. A chord's departure along the axis is nil, so the turning radius is the whole of it.
inline double sampling_resolution(const std::vector<Eigen::Vector3d> &curve, const std::vector<Eigen::Vector3d> &ends)
{
    const Eigen::Vector3d along = (ends.back() - ends.front()).normalized();
    const Eigen::Vector3d off   = curve.front() - ends.front();
    const double radius         = (off - off.dot(along) * along).norm();
    const double step           = 2.0 * rigid_motion::screw_window::angle_limit_radians / static_cast<double>(curve.size() - 1u);

    return radius * (1.0 - std::cos(0.5 * step));
}

}

#endif
