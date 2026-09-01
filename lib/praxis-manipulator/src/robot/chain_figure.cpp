#include "robot/chain_figure.h"
#include "robot/joint_decoration.h"

#include <threepp/objects/Mesh.hpp>

#include <threepp/math/Color.hpp>

#include <threepp/geometries/SphereGeometry.hpp>
#include <threepp/geometries/CylinderGeometry.hpp>

#include <threepp/materials/MeshPhongMaterial.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <span>
#include <string>
#include <memory>
#include <cstddef>
#include <utility>

namespace praxis::manipulator {

namespace {

constexpr double segment_girth                 = 0.008; // the diameter of a segment, in metres
constexpr double mark_radius_multiple          = 1.75;  // of the segment's radius
constexpr threepp::Color::ColorName chain_tone = threepp::Color::black;

void place_segment(threepp::Object3D &drawn, const Eigen::Vector3d &from, const Eigen::Vector3d &to)
{
    const Eigen::Vector3d along = to - from;
    const double length         = along.norm();

    transform placed         = transform::Identity();
    placed.block<3, 1>(0, 3) = 0.5 * (from + to);
    if(length > 0.0)
        placed.topLeftCorner<3, 3>() = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitY(), along / length).toRotationMatrix();

    write_placement(drawn, placed);
    drawn.scale.y = static_cast<float>(length);
    drawn.visible = length > 0.0;
}

void place_mark(threepp::Object3D &drawn, const Eigen::Vector3d &at)
{
    transform placed         = transform::Identity();
    placed.block<3, 1>(0, 3) = at;

    write_placement(drawn, placed);
}

}

std::shared_ptr<threepp::Material> chain_material(bool told)
{
    return threepp::MeshPhongMaterial::create({{"flatShading", true}, {"color", told ? threepp::Color(selected_joint_tone) : threepp::Color(chain_tone)}});
}

// A cylinder is built along the renderer's +Y, which is why a segment is turned from there while an
// axis line, built along +Z, is turned from there.
std::shared_ptr<threepp::Object3D> chain_segment_object(std::string name, std::shared_ptr<threepp::Material> tone)
{
    const auto radius = static_cast<float>(segment_girth / 2.0);

    auto drawn  = threepp::Mesh::create(threepp::CylinderGeometry::create(radius, radius, 1.f), std::move(tone));
    drawn->name = std::move(name);

    return drawn;
}

std::shared_ptr<threepp::Object3D> joint_mark_object(std::string name, std::shared_ptr<threepp::Material> tone)
{
    const auto radius = static_cast<float>(mark_radius_multiple * segment_girth / 2.0);

    auto drawn  = threepp::Mesh::create(threepp::SphereGeometry::create(radius), std::move(tone));
    drawn->name = std::move(name);

    return drawn;
}

void place_chain_figure(std::span<const std::shared_ptr<threepp::Object3D>> segments, std::span<const std::shared_ptr<threepp::Object3D>> marks, std::span<const Eigen::Vector3d> points)
{
    for(std::size_t at = 0; at < segments.size() && at + 1 < points.size(); ++at)
        place_segment(*segments[at], points[at], points[at + 1]);

    for(std::size_t joint = 0; joint < marks.size() && joint + 1 < points.size(); ++joint)
        place_mark(*marks[joint], points[joint + 1]);
}

}
