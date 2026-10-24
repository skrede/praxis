#include "surface_figures.h"

#include <threepp/objects/Line.hpp>
#include <threepp/objects/LineSegments.hpp>

#include <threepp/math/Color.hpp>
#include <threepp/math/Matrix4.hpp>

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

void lay_over(threepp::Object3D &drawn, threepp::Material &tone)
{
    tone.depthTest    = false;
    tone.depthWrite   = false;
    drawn.renderOrder = over_the_field;
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
    std::shared_ptr<threepp::Object3D> drawn = occluded_lines(std::move(name), tone, points, joined);
    lay_over(*drawn, *drawn->material());

    return drawn;
}

std::shared_ptr<threepp::Object3D> occluded_lines(std::string name, std::uint32_t tone, std::size_t points, bool joined)
{
    auto material = threepp::LineBasicMaterial::create({{"color", threepp::Color(tone)}});
    std::shared_ptr<threepp::Object3D> drawn;
    if(joined)
        drawn = threepp::Line::create(room_for(points, false), material);
    else
        drawn = threepp::LineSegments::create(room_for(points, false), material);
    drawn->frustumCulled = false;
    drawn->visible       = false;
    drawn->name          = std::move(name);

    return drawn;
}

std::shared_ptr<threepp::Mesh> overlay_point(std::string name, std::uint32_t tone, std::shared_ptr<threepp::BufferGeometry> sphere)
{
    auto material   = threepp::MeshPhongMaterial::create();
    material->color = threepp::Color(tone);
    auto drawn      = threepp::Mesh::create(std::move(sphere), material);
    lay_over(*drawn, *material);
    drawn->frustumCulled = false;
    drawn->visible       = false;
    drawn->name          = std::move(name);

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

void lay_over_field(threepp::Object3D &drawn, bool over)
{
    drawn.traverse(
            [over](threepp::Object3D &part)
            {
                part.renderOrder = over ? over_the_field : 0;
                if(const std::shared_ptr<threepp::Material> tone = part.material())
                {
                    tone->transparent = over;
                    tone->depthTest   = !over;
                    tone->depthWrite  = !over;
                }
            });
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

void hold_marks(threepp::Object3D &under, std::shared_ptr<threepp::InstancedMesh> &marks, std::size_t count, const marks_look &look)
{
    if(marks != nullptr && static_cast<std::size_t>(marks->instanceMatrix()->count()) == count)
        return;

    if(marks != nullptr)
        under.remove(*marks);
    marks = nullptr;
    if(count == 0u)
        return;

    marks       = threepp::InstancedMesh::create(look.sphere, look.material, count);
    marks->name = look.name;
    // threepp picks the shader by whether instance colors exist at the first draw, so they exist from the start.
    marks->setColorAt(0u, threepp::Color(look.tone));
    under.add(marks);
}

void write_marks(threepp::InstancedMesh &marks, std::span<const Eigen::Vector3d> centers, std::span<const std::uint32_t> tones)
{
    for(std::size_t j = 0; j < centers.size(); ++j)
    {
        const Eigen::Vector3d &at = centers[j];
        marks.setMatrixAt(j, threepp::Matrix4().makeTranslation(static_cast<float>(at.x()), static_cast<float>(at.y()), static_cast<float>(at.z())));
        marks.setColorAt(j, threepp::Color(tones[j]));
    }
    marks.setCount(centers.size());
    marks.instanceMatrix()->needsUpdate();
    marks.instanceColor()->needsUpdate();
    marks.computeBoundingSphere();
}

}
