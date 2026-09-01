#include "praxis/presets/twist_axis.h"

#include "praxis/rigid_motion/axes.h"
#include "praxis/rigid_motion/screw_window.h"
#include "praxis/rigid_motion/frame_stencil.h"
#include "praxis/rigid_motion/twist_axis_window.h"

#include "praxis/scene/imgui_window.h"
#include "praxis/scene/labeled_value_window.h"

#include <Eigen/Core>

#include <threepp/objects/Line.hpp>

#include <threepp/math/Color.hpp>

#include <threepp/core/BufferGeometry.hpp>

#include <threepp/materials/LineBasicMaterial.hpp>

#include <cmath>
#include <memory>
#include <vector>
#include <cstddef>
#include <numbers>
#include <optional>

namespace praxis::presets {

namespace {

constexpr double body_edge             = 0.30;
constexpr double axis_half_length      = 2.5;
constexpr float resting_height         = 0.5f;
constexpr float initial_pitch          = 0.12f;
constexpr std::size_t samples_per_turn = 48;

// The object carrying the travelled curve, which stands after the two the controls address by index.
constexpr std::size_t path_object = 2;

// A screw axis carries its direction and, in its second three components, the moment of that
// direction about the origin: for an axis through the point q along the unit direction s with a
// pitch h metres per radian, that moment is -s x q + h s. The components below are what the world
// z-direction through the point (resting_height, 0, 0) at that pitch spells.
rigid_motion::twist_axis_window::settings opening()
{
    return rigid_motion::twist_axis_window::settings{Eigen::Vector3f{0.f, 0.f, 1.f}, Eigen::Vector3f{0.f, -resting_height, initial_pitch}, 0.f};
}

struct located_axis
{
    Eigen::Vector3d along;
    Eigen::Vector3d through;
};

// s x v recovers the point on a turning axis nearest the origin from that moment. An axis that only
// slides names a direction and no located line and is drawn through the origin; one that does
// neither names no line at all.
std::optional<located_axis> located(const screw_axis &named)
{
    const Eigen::Vector3d turning = named.head<3>();
    const Eigen::Vector3d sliding = named.tail<3>();
    if(!turning.isZero())
    {
        const Eigen::Vector3d along = turning.normalized();

        return located_axis{along, along.cross(sliding)};
    }

    if(!sliding.isZero())
        return located_axis{sliding.normalized(), Eigen::Vector3d::Zero()};

    return std::nullopt;
}

threepp::Vector3 drawn_at(const Eigen::Vector3d &point)
{
    return threepp::Vector3{static_cast<float>(point.x()), static_cast<float>(point.y()), static_cast<float>(point.z())};
}

// Built where the axis runs, in the frame the stencil's poses are written in: the object carrying
// the line is never placed, so what the line says about the axis is what was built into it.
std::shared_ptr<threepp::Object3D> axis_line(const located_axis &where)
{
    const std::vector<threepp::Vector3> ends{drawn_at(where.through - axis_half_length * where.along), drawn_at(where.through + axis_half_length * where.along)};

    auto drawn = threepp::BufferGeometry::create();
    drawn->setFromPoints(ends);

    return threepp::Line::create(drawn, threepp::LineBasicMaterial::create({{"color", threepp::Color::darkviolet}}));
}

// The path the moving object travels, in the same frame the axis line is built in: the pose that
// object is driven from, carried through the exponential driving it over the whole angle the
// controls reach, so no angle they can command lies off the drawn curve.
std::shared_ptr<threepp::Object3D> traversed_path(const rigid_motion::screw_ops &screw, const screw_axis &about, const transform &start)
{
    const double turns         = rigid_motion::screw_window::angle_limit_radians / std::numbers::pi;
    const auto samples         = static_cast<std::size_t>(std::llround(turns * static_cast<double>(samples_per_turn)));
    const Eigen::Vector3d seed = start.block<3, 1>(0, 3);

    std::vector<threepp::Vector3> points;
    points.reserve(samples + 1);
    for(std::size_t step = 0; step <= samples; ++step)
    {
        const double angle       = rigid_motion::screw_window::angle_limit_radians * (2.0 * static_cast<double>(step) / static_cast<double>(samples) - 1.0);
        const transform put      = screw.matrix_exponential_screw(about, angle);
        const Eigen::Vector3d at = put.topLeftCorner<3, 3>() * seed + put.block<3, 1>(0, 3);

        points.push_back(drawn_at(at));
    }

    auto drawn = threepp::BufferGeometry::create();
    drawn->setFromPoints(points);

    return threepp::Line::create(drawn, threepp::LineBasicMaterial::create({{"color", threepp::Color::darkorange}}));
}

// Both drawn objects are rebuilt from the one call, so the line naming the axis and the curve
// travelled about it cannot disagree about which axis that is.
rigid_motion::twist_axis_window::axis_route axis_into(rigid_motion::frame_stencil &target, const rigid_motion::screw_ops &screw, const transform &start)
{
    return [&target, screw, start](const std::optional<screw_axis> &named)
    {
        const std::optional<located_axis> where = named ? located(*named) : std::optional<located_axis>();
        const rigid_motion::object_body nothing{rigid_motion::body_shape::none, 0.0, nullptr};

        target.set_body(rigid_motion::twist_axis_window::axis_object, where ? rigid_motion::object_body{rigid_motion::body_shape::mesh, 0.0, axis_line(*where)} : nothing);
        target.set_body(path_object, where ? rigid_motion::object_body{rigid_motion::body_shape::mesh, 0.0, traversed_path(screw, *named, start)} : nothing);
    };
}

std::vector<rigid_motion::stencil_object> twist_axis_objects()
{
    const rigid_motion::object_body nothing{rigid_motion::body_shape::none, 0.0, nullptr};

    return {rigid_motion::stencil_object{"Axis", rigid_motion::axes_settings{false}, nothing},
            rigid_motion::stencil_object{"Body", rigid_motion::axes_settings{}, rigid_motion::object_body{rigid_motion::body_shape::cube, body_edge, nullptr}},
            rigid_motion::stencil_object{"Path", rigid_motion::axes_settings{false}, nothing}};
}

}

std::shared_ptr<scene::preset> twist_axis_preset(const scene::preset_site &site, const rigid_motion::capabilities &motions)
{
    auto body = std::make_shared<rigid_motion::frame_stencil>(site.scene, twist_axis_objects(), motions.frame);

    // The pose the controls take as the one the exponential is applied to, so the curve is sampled
    // from the term the moving object is driven by rather than from a second spelling of it.
    const transform start = motions.frame.transformation_matrix_from_position(Eigen::Vector3d{0.0, 0.0, resting_height});

    body->set_pose(rigid_motion::twist_axis_window::body_object, start);

    const auto driving = std::make_shared<rigid_motion::twist_axis_window>("Twist", *body, motions, axis_into(*body, motions.screw, start), opening());
    const std::vector<std::shared_ptr<scene::imgui_window>> windows{driving,
                                                                    std::make_shared<scene::labeled_value_window>("Screw axis", nullptr, [driving] { return driving->reading(); })};

    return std::make_shared<scene::preset>(body, windows, site.add_window, site.remove_window);
}

}
