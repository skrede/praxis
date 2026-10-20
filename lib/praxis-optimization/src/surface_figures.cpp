#include "surface_figures.h"

#include <threepp/objects/Line.hpp>
#include <threepp/objects/LineSegments.hpp>

#include <threepp/core/BufferAttribute.hpp>

#include <threepp/materials/LineBasicMaterial.hpp>
#include <threepp/materials/MeshPhongMaterial.hpp>

#include <Eigen/Geometry>

#include <span>
#include <array>
#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <algorithm>

namespace praxis::optimization {

namespace {

// Drawn after the field, which has written its depths by then.
constexpr int over_the_field = 1;

std::shared_ptr<threepp::BufferGeometry> room_for(std::size_t points, bool lit)
{
    auto geometry = threepp::BufferGeometry::create();
    geometry->setAttribute("position", threepp::FloatBufferAttribute::create(std::vector<float>(3u * points), 3));
    if(lit)
        geometry->setAttribute("normal", threepp::FloatBufferAttribute::create(std::vector<float>(3u * points), 3));
    geometry->setDrawRange(0, 0);

    return geometry;
}

void lay_over(threepp::Object3D &drawn, threepp::Material &tone, std::string name)
{
    tone.depthTest      = false;
    tone.depthWrite     = false;
    drawn.renderOrder   = over_the_field;
    drawn.frustumCulled = false;
    drawn.visible       = false;
    drawn.name          = std::move(name);
}

std::size_t written(threepp::FloatBufferAttribute &into, std::span<const Eigen::Vector3d> points, const field_placement &placed)
{
    const std::size_t count = std::min(points.size(), into.array().size() / 3u);
    for(std::size_t at = 0; at < count; ++at)
    {
        const Eigen::Vector3d standing = placed.at(points[at].x(), points[at].y(), points[at].z());
        for(int axis = 0; axis < 3; ++axis)
            into.array()[3u * at + static_cast<std::size_t>(axis)] = static_cast<float>(standing[axis]);
    }
    into.needsUpdate();

    return count;
}

}

std::shared_ptr<threepp::Object3D> overlay_lines(std::string name, std::uint32_t tone, std::size_t points, bool joined)
{
    auto material = threepp::LineBasicMaterial::create({{"color", threepp::Color(tone)}});
    std::shared_ptr<threepp::Object3D> drawn;
    if(joined)
        drawn = threepp::Line::create(room_for(points, false), material);
    else
        drawn = threepp::LineSegments::create(room_for(points, false), material);
    lay_over(*drawn, *material, std::move(name));

    return drawn;
}

std::shared_ptr<threepp::Mesh> overlay_point(std::string name, std::uint32_t tone, std::shared_ptr<threepp::BufferGeometry> sphere)
{
    auto material   = threepp::MeshPhongMaterial::create();
    material->color = threepp::Color(tone);
    auto drawn      = threepp::Mesh::create(std::move(sphere), material);
    lay_over(*drawn, *material, std::move(name));

    return drawn;
}

std::shared_ptr<threepp::Mesh> plane_patch(std::string name, std::uint32_t tone, std::size_t corners, bool wired)
{
    auto material       = threepp::MeshPhongMaterial::create();
    material->color     = threepp::Color(tone);
    material->side      = threepp::Side::Double;
    material->wireframe = wired;

    auto drawn           = threepp::Mesh::create(room_for(3u * (corners - 2u), true), material);
    drawn->name          = std::move(name);
    drawn->frustumCulled = false;
    drawn->visible       = false;

    return drawn;
}

void write_points(threepp::Object3D &drawn, std::span<const Eigen::Vector3d> points, const field_placement &placed)
{
    threepp::BufferGeometry &shaped = *drawn.geometry();
    const std::size_t count         = written(*shaped.getAttribute<float>("position"), points, placed);
    shaped.setDrawRange(0, static_cast<int>(count));
    drawn.visible = count > 0u;
}

// A triangle fan about the first corner, every vertex given the polygon's one normal.
void write_polygon(threepp::Mesh &drawn, std::span<const Eigen::Vector3d> polygon, const field_placement &placed)
{
    std::vector<Eigen::Vector3d> fan;
    for(std::size_t at = 1; at + 1 < polygon.size(); ++at)
        fan.insert(fan.end(), {polygon[0], polygon[at], polygon[at + 1]});
    write_points(drawn, fan, placed);
    if(fan.empty())
        return;

    std::array<Eigen::Vector3d, 3> standing;
    for(std::size_t at = 0; at < standing.size(); ++at)
        standing[at] = placed.at(fan[at].x(), fan[at].y(), fan[at].z());
    const Eigen::Vector3d up = (standing[1] - standing[0]).cross(standing[2] - standing[0]).normalized();
    auto *normals            = drawn.geometry()->getAttribute<float>("normal");
    for(std::size_t at = 0; at < normals->array().size(); ++at)
        normals->array()[at] = static_cast<float>(up[static_cast<int>(at % 3u)]);
    normals->needsUpdate();
}

void place_point(threepp::Object3D &drawn, const Eigen::Vector3d &point, const field_placement &placed)
{
    const Eigen::Vector3d standing = placed.at(point.x(), point.y(), point.z());
    drawn.position.set(static_cast<float>(standing.x()), static_cast<float>(standing.y()), static_cast<float>(standing.z()));
    drawn.visible = true;
}

}
