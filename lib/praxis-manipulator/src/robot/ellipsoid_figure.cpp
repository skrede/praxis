#include "robot/capped_body.h"
#include "robot/ellipsoid_figure.h"
#include "robot/joint_decoration.h"

#include "praxis/rigid_motion/types.h"

#include <threepp/objects/Line.hpp>
#include <threepp/objects/Mesh.hpp>

#include <threepp/math/Vector3.hpp>

#include <threepp/core/BufferGeometry.hpp>
#include <threepp/core/BufferAttribute.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <any>
#include <limits>
#include <string>
#include <memory>
#include <vector>
#include <cstddef>
#include <utility>
#include <optional>

namespace praxis::manipulator {

namespace {

constexpr const char *shape_memo = "praxis.ellipsoid.shape";

struct written_shape
{
    Eigen::Vector3d semi_axes;
    std::optional<double> cap;
};

// Where the cut stands once the ellipsoid is the unit sphere. A semi-axis that is not positive
// carries the cut out of reach rather than dividing by it, so a flattened body is cut on the axes
// it still has and left alone on the one it has lost.
Eigen::Vector3d half_widths_of(const Eigen::Vector3d &semi_axes, const std::optional<double> &cap)
{
    Eigen::Vector3d half = Eigen::Vector3d::Constant(std::numeric_limits<double>::infinity());
    if(!cap)
        return half;

    for(Eigen::Index axis = 0; axis < 3; ++axis)
        if(semi_axes[axis] > 0.0)
            half[axis] = *cap / semi_axes[axis];

    return half;
}

bool already_shaped(const threepp::Object3D &drawn, const Eigen::Vector3d &semi_axes, const std::optional<double> &cap)
{
    const auto held = drawn.userData.find(shape_memo);
    if(held == drawn.userData.end())
        return false;

    const auto *last = std::any_cast<written_shape>(&held->second);

    return last != nullptr && last->cap == cap && last->semi_axes.cwiseEqual(semi_axes).all();
}

// The soup is built on the unit sphere, so this is the one place the semi-axes are carried in. The
// vertices past the ones in use go back onto the origin, which lies inside every body, so nothing
// read off the whole buffer is left over from a shape that is gone.
void stretch(std::vector<float> &into, std::size_t vertices, std::size_t held, const Eigen::Vector3d &semi_axes)
{
    for(std::size_t at = 0u; at < 3u * vertices && at < into.size(); ++at)
        into[at] *= static_cast<float>(semi_axes[static_cast<Eigen::Index>(at % 3u)]);

    for(std::size_t at = 3u * vertices; at < 3u * held && at < into.size(); ++at)
        into[at] = 0.f;
}

Eigen::Vector3d corner_of(const std::vector<float> &from, std::size_t at)
{
    return Eigen::Vector3d(from[at], from[at + 1u], from[at + 2u]);
}

// The bodies are drawn flat-shaded, so one normal per triangle stands at each of its three corners.
void face_normals(const std::vector<float> &from, std::vector<float> &into, std::size_t vertices)
{
    for(std::size_t at = 0u; at + 8u < 3u * vertices && at + 8u < into.size(); at += 9u)
    {
        const Eigen::Vector3d across = (corner_of(from, at + 3u) - corner_of(from, at)).cross(corner_of(from, at + 6u) - corner_of(from, at));
        const double reach           = across.norm();
        const Eigen::Vector3d out    = reach > 0.0 ? Eigen::Vector3d(across / reach) : Eigen::Vector3d::Zero();
        for(std::size_t corner = 0u; corner < 9u; corner += 3u)
        {
            into[at + corner]      = static_cast<float>(out.x());
            into[at + corner + 1u] = static_cast<float>(out.y());
            into[at + corner + 2u] = static_cast<float>(out.z());
        }
    }
}

// The soup the half-widths leave, stretched onto the semi-axes and given its normals, with the
// drawn range naming how much of a buffer that is never replaced it filled.
void write_body(threepp::BufferGeometry &shaped, const Eigen::Vector3d &semi_axes, const std::optional<double> &cap)
{
    auto *placed           = shaped.getAttribute<float>("position");
    auto *facing           = shaped.getAttribute<float>("normal");
    const std::size_t held = static_cast<std::size_t>(shaped.drawRange.count);

    std::vector<float> &into   = placed->array();
    const std::size_t vertices = cap_unit_body(half_widths_of(semi_axes, cap), into);
    stretch(into, vertices, held, semi_axes);
    face_normals(into, facing->array(), vertices);

    shaped.setDrawRange(0, static_cast<int>(vertices));
    placed->needsUpdate();
    facing->needsUpdate();
}

// The soup is drawn through an index of its own order, so the renderer builds the wireframe of the
// angular body once rather than on every frame it is drawn.
std::vector<unsigned int> in_order(std::size_t room)
{
    std::vector<unsigned int> order(room);
    for(std::size_t at = 0u; at < room; ++at)
        order[at] = static_cast<unsigned int>(at);

    return order;
}

}

std::shared_ptr<threepp::Object3D> ellipsoid_object(std::string name, std::shared_ptr<threepp::Material> tone)
{
    const std::size_t room = capped_body_vertex_bound();

    const std::shared_ptr<threepp::BufferGeometry> geometry = threepp::BufferGeometry::create();
    geometry->setAttribute("position", threepp::FloatBufferAttribute::create(std::vector<float>(3u * room), 3));
    geometry->setAttribute("normal", threepp::FloatBufferAttribute::create(std::vector<float>(3u * room), 3));
    geometry->setIndex(in_order(room));
    write_body(*geometry, Eigen::Vector3d::Ones(), std::nullopt);

    auto drawn  = threepp::Mesh::create(geometry, std::move(tone));
    drawn->name = std::move(name);

    return drawn;
}

void shape_ellipsoid(threepp::Object3D &drawn, const Eigen::Vector3d &semi_axes, std::optional<double> cap)
{
    const std::shared_ptr<threepp::BufferGeometry> shaped = drawn.geometry();
    const bool writable                                   = shaped != nullptr && shaped->getAttribute<float>("position") != nullptr && shaped->getAttribute<float>("normal") != nullptr;
    if(!writable || already_shaped(drawn, semi_axes, cap))
        return;

    write_body(*shaped, semi_axes, cap);
    drawn.userData[shape_memo] = written_shape{semi_axes, cap};
}

void place_ellipsoid(threepp::Object3D &drawn, const Eigen::Matrix3d &axes, const Eigen::Vector3d &at)
{
    transform placed             = transform::Identity();
    placed.topLeftCorner<3, 3>() = axes;
    placed.block<3, 1>(0, 3)     = at;

    write_placement(drawn, placed);
    drawn.scale.set(1.f, 1.f, 1.f);
}

// A line's width is not settable through any OpenGL core profile, so the drawing is one pixel wide
// whatever is asked of the material, and the tone is the only thing carrying it.
std::shared_ptr<threepp::Object3D> continuation_object(std::string name, std::shared_ptr<threepp::Material> tone)
{
    const std::vector<threepp::Vector3> ends{threepp::Vector3{0.f, 0.f, 0.f}, threepp::Vector3{0.f, 0.f, 1.f}};

    auto geometry = threepp::BufferGeometry::create();
    geometry->setFromPoints(ends);

    auto drawn  = threepp::Line::create(geometry, std::move(tone));
    drawn->name = std::move(name);

    return drawn;
}

void place_continuation(threepp::Object3D &drawn, const Eigen::Vector3d &from, const Eigen::Vector3d &along, double length)
{
    const double reach = along.norm();

    transform placed         = transform::Identity();
    placed.block<3, 1>(0, 3) = from;
    if(reach > 0.0)
        placed.topLeftCorner<3, 3>() = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), along / reach).toRotationMatrix();

    write_placement(drawn, placed);
    drawn.scale.z = static_cast<float>(length);
    drawn.visible = length > 0.0 && reach > 0.0;
}

}
