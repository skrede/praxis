#include "praxis/presets/screw.h"

#include "praxis/rigid_motion/axes.h"
#include "praxis/rigid_motion/screw_window.h"
#include "praxis/rigid_motion/frame_stencil.h"

#include <Eigen/Core>

#include <threepp/objects/Line.hpp>
#include <threepp/objects/Mesh.hpp>
#include <threepp/objects/Group.hpp>

#include <threepp/math/Color.hpp>

#include <threepp/core/BufferGeometry.hpp>

#include <threepp/geometries/SphereGeometry.hpp>

#include <threepp/materials/LineBasicMaterial.hpp>
#include <threepp/materials/MeshPhongMaterial.hpp>

#include <cmath>
#include <vector>
#include <memory>
#include <string>
#include <cstddef>
#include <numbers>
#include <optional>

namespace praxis::presets {

namespace {

constexpr double body_edge             = 0.30;
constexpr double body_standoff         = 0.30;
constexpr double axis_half_length      = 2.5;
constexpr float resting_height         = 0.5f;
constexpr float initial_pitch          = 0.12f;
constexpr std::size_t samples_per_turn = 64;

// The radius of the mark standing on the axis point, as a multiple of the thickness the triads are
// drawn at.
constexpr double mark_radius_multiple = 0.50;
constexpr double mark_radius          = rigid_motion::axes_settings{}.axis_thickness * mark_radius_multiple;

// The object carrying the located axis point, which stands after the three the controls address by
// their own indices.
constexpr std::size_t located_object = 3;

const threepp::Color located_tone = threepp::Color::black;

rigid_motion::screw_window::settings upright_axis()
{
    return rigid_motion::screw_window::settings{Eigen::Vector3f{0.f, 0.f, resting_height}, Eigen::Vector3f{0.f, 0.f, 1.f}, initial_pitch, 0.f};
}

threepp::Vector3 drawn_at(const Eigen::Vector3d &point)
{
    return threepp::Vector3{static_cast<float>(point.x()), static_cast<float>(point.y()), static_cast<float>(point.z())};
}

// Drawn along the z-axis through the origin, which is the axis object's own frame: the object's
// placement is what carries the line onto the named point along the named direction.
std::shared_ptr<threepp::Object3D> axis_line()
{
    const std::vector<threepp::Vector3> ends{threepp::Vector3{0.f, 0.f, -static_cast<float>(axis_half_length)}, threepp::Vector3{0.f, 0.f, static_cast<float>(axis_half_length)}};

    auto drawn = threepp::BufferGeometry::create();
    drawn->setFromPoints(ends);

    return threepp::Line::create(drawn, threepp::LineBasicMaterial::create({{"color", threepp::Color::darkviolet}}));
}

// The point the axis passes through, drawn where it stands: a connector out of the origin of the
// frame the poses are written in, and a mark at the end of it. The object carrying the two is never
// placed, so what they say about the point is what was built into them.
std::shared_ptr<threepp::Object3D> located_point(const Eigen::Vector3d &at)
{
    const std::vector<threepp::Vector3> ends{threepp::Vector3{}, drawn_at(at)};

    auto span = threepp::BufferGeometry::create();
    span->setFromPoints(ends);

    auto mark      = threepp::Mesh::create(threepp::SphereGeometry::create(static_cast<float>(mark_radius)),
                                           threepp::MeshPhongMaterial::create({{"flatShading", true}, {"color", located_tone}}));
    mark->name     = "point";
    mark->position = drawn_at(at);

    auto drawn = threepp::Group::create();
    drawn->add(threepp::Line::create(span, threepp::LineBasicMaterial::create({{"color", located_tone}})));
    drawn->add(mark);

    return drawn;
}

std::vector<rigid_motion::stencil_object> screw_objects()
{
    const rigid_motion::object_body nothing{rigid_motion::body_shape::none, 0.0, nullptr};

    return {rigid_motion::stencil_object{"Axis", rigid_motion::axes_settings{false}, rigid_motion::object_body{rigid_motion::body_shape::mesh, 0.0, axis_line()}},
            rigid_motion::stencil_object{"Body", rigid_motion::axes_settings{}, rigid_motion::object_body{rigid_motion::body_shape::cube, body_edge, nullptr}},
            rigid_motion::stencil_object{"Threads", rigid_motion::axes_settings{false}, nothing}, rigid_motion::stencil_object{"Point", rigid_motion::axes_settings{false}, nothing}};
}

// The path the moving body travels, in the same frame the located point is built in: the pose that
// body is driven from, carried through the exponential driving it over the whole angle the controls
// reach, so no angle they can command lies off the drawn curve.
std::shared_ptr<threepp::Object3D> traversed_thread(const rigid_motion::screw_ops &screw, const screw_axis &about, const transform &start)
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

rigid_motion::screw_window::axis_route axis_into(rigid_motion::frame_stencil &target, const rigid_motion::screw_ops &screw, const transform &start)
{
    return [&target, screw, start](const rigid_motion::screw_window::settings &shown, const std::optional<screw_axis> &named)
    {
        const rigid_motion::object_body nothing{rigid_motion::body_shape::none, 0.0, nullptr};

        target.set_body(rigid_motion::screw_window::thread_object,
                        named ? rigid_motion::object_body{rigid_motion::body_shape::mesh, 0.0, traversed_thread(screw, *named, start)} : nothing);
        target.set_body(located_object, named ? rigid_motion::object_body{rigid_motion::body_shape::mesh, 0.0, located_point(shown.point.cast<double>())} : nothing);
    };
}

}

std::shared_ptr<scene::preset> screw_preset(const scene::preset_site &site, const rigid_motion::capabilities &motions)
{
    auto body = std::make_shared<rigid_motion::frame_stencil>(site.scene, screw_objects(), motions.frame, rigid_motion::fixed_frame{"Space", rigid_motion::axes_settings{}});

    // The pose the controls take as the one the exponential is applied to, so the curve is sampled
    // from the term the moving body is driven by rather than from a second spelling of it.
    const transform start = motions.frame.transformation_matrix_from_position(Eigen::Vector3d{body_standoff, 0.0, resting_height});

    body->set_pose(rigid_motion::screw_window::body_object, start);

    const std::vector<std::shared_ptr<scene::imgui_window>> windows{
            std::make_shared<rigid_motion::screw_window>("Screw", *body, motions, axis_into(*body, motions.screw, start), upright_axis()),
    };

    return std::make_shared<scene::preset>(body, windows, site.add_window, site.remove_window);
}

}
