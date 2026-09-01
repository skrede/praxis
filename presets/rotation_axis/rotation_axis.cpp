#include "rotation_arc.h"
#include "rotation_arrow.h"

#include "praxis/presets/rotation_axis.h"

#include "praxis/rigid_motion/axes.h"
#include "praxis/rigid_motion/types.h"
#include "praxis/rigid_motion/frame_stencil.h"
#include "praxis/rigid_motion/rotation_axis_window.h"

#include "praxis/scene/imgui_window.h"

#include <threepp/math/Color.hpp>

#include <Eigen/Core>

#include <array>
#include <memory>
#include <vector>
#include <cstddef>
#include <optional>

namespace praxis::presets {

namespace {

// The two objects the scenario carries its own arrows on, standing after the one the controls drive,
// and the three carrying the arcs the frame's own arrows trace.
constexpr std::size_t axis_object       = 1;
constexpr std::size_t coordinate_object = 2;
constexpr std::size_t arc_x_object      = 3;
constexpr std::size_t arc_y_object      = 4;
constexpr std::size_t arc_z_object      = 5;

// The settings the frame is composed with, read once so the reach its own arrows are drawn at, the
// radii the arcs are traced at and the length the arrow standing for the axis is drawn at move
// together.
constexpr rigid_motion::axes_settings frame_axes{};

// The girth of each arrow, as a multiple of the thickness the triads are drawn at.
constexpr double unit_girth_multiple       = 0.40;
constexpr double coordinate_girth_multiple = 0.20;
constexpr double unit_girth                = frame_axes.axis_thickness * unit_girth_multiple;
constexpr double coordinate_girth          = frame_axes.axis_thickness * coordinate_girth_multiple;

static_assert(unit_girth > coordinate_girth,
              "the arrow standing for the axis is drawn thicker than the arrow standing for the coordinate, at the stem and at the head alike, so "
              "neither of the two stands hidden inside the other on the ray they share");

const threepp::Color unit_tone       = threepp::Color::darkviolet;
const threepp::Color coordinate_tone = threepp::Color::mediumblue;

rigid_motion::rotation_axis_window::settings opening()
{
    return rigid_motion::rotation_axis_window::settings{Eigen::Vector3f{1.f, 1.f, 1.f}.normalized(), 1.f, true, true, true, true};
}

std::vector<rigid_motion::stencil_object> rotation_axis_objects()
{
    const rigid_motion::object_body nothing{rigid_motion::body_shape::none, 0.0, nullptr};
    const rigid_motion::axes_settings bare{false};

    return {rigid_motion::stencil_object{"Frame", frame_axes, nothing}, rigid_motion::stencil_object{"Axis", bare, nothing},  rigid_motion::stencil_object{"Coordinate", bare, nothing},
            rigid_motion::stencil_object{"Arc x", bare, nothing},       rigid_motion::stencil_object{"Arc y", bare, nothing}, rigid_motion::stencil_object{"Arc z", bare, nothing}};
}

// The end of one of the moving frame's own arrows, in that frame's own coordinates, and the object
// carrying the arc it traces. The tone is the arrow's own, so which arc belongs to which arrow needs
// nothing beside it to say so.
struct traced_arc
{
    std::size_t object;
    Eigen::Vector3d carried;
    threepp::Color tone;
};

std::array<traced_arc, 3> arcs_traced()
{
    constexpr double reach = frame_axes.axis_length;

    return {traced_arc{arc_x_object, Eigen::Vector3d{reach, 0.0, 0.0}, threepp::Color::red}, traced_arc{arc_y_object, Eigen::Vector3d{0.0, reach, 0.0}, threepp::Color::green},
            traced_arc{arc_z_object, Eigen::Vector3d{0.0, 0.0, reach}, threepp::Color::blue}};
}

// Every point of an arc moves when either the axis or the angle moves, so an arc is rebuilt on a
// settled change rather than placed.
void rebuild_arcs(rigid_motion::frame_stencil &target, const rigid_motion::screw_ops &screw, const transform &start, const rigid_motion::rotation_axis_window::settings &shown,
                  const std::optional<Eigen::Vector3d> &named)
{
    for(const traced_arc &traced : arcs_traced())
    {
        std::shared_ptr<threepp::Object3D> drawn;
        if(named && shown.arc_shown)
            drawn = traversed_arc(screw, *named, static_cast<double>(shown.angle_radians), start, traced.carried, traced.tone);

        target.set_body(traced.object,
                        drawn != nullptr ? rigid_motion::object_body{rigid_motion::body_shape::mesh, 0.0, drawn}
                                         : rigid_motion::object_body{rigid_motion::body_shape::none, 0.0, nullptr});
    }
}

// Both arrows and all three arcs stand on the one ray the controls name, so they are built from the
// one call and cannot disagree about which direction that is. The switches are a second condition on
// top of what the geometry itself decides.
rigid_motion::rotation_axis_window::axis_route drawings_into(rigid_motion::frame_stencil &target, const rigid_motion::screw_ops &screw, const transform &start, const drawn_arrow &unit,
                                                             const drawn_arrow &coordinate)
{
    return [&target, screw, start, unit, coordinate](const rigid_motion::rotation_axis_window::settings &shown, const std::optional<Eigen::Vector3d> &named)
    {
        // Both arrows are drawn through the one length, in metres: the arrow standing for the axis
        // at the length the frame's own arrows are drawn at, the arrow standing for the coordinate
        // at that same length times the angle.
        const Eigen::Vector3d along = named.value_or(Eigen::Vector3d::Zero());

        place_arrow(unit, along * frame_axes.axis_length);
        place_arrow(coordinate, along * (frame_axes.axis_length * static_cast<double>(shown.angle_radians)));
        rebuild_arcs(target, screw, start, shown, named);

        unit.object->visible       = unit.object->visible && shown.axis_shown;
        coordinate.object->visible = coordinate.object->visible && shown.coordinate_shown;
        target.set_axes_shown(rigid_motion::rotation_axis_window::frame_object, shown.frame_shown);
    };
}

}

std::shared_ptr<scene::preset> rotation_axis_preset(const scene::preset_site &site, const rigid_motion::capabilities &motions)
{
    auto body = std::make_shared<rigid_motion::frame_stencil>(site.scene, rotation_axis_objects(), motions.frame);

    const drawn_arrow unit       = arrow_object("unit", unit_tone, unit_girth);
    const drawn_arrow coordinate = arrow_object("coordinate", coordinate_tone, coordinate_girth);

    body->set_body(axis_object, rigid_motion::object_body{rigid_motion::body_shape::mesh, 0.0, unit.object});
    body->set_body(coordinate_object, rigid_motion::object_body{rigid_motion::body_shape::mesh, 0.0, coordinate.object});

    // The pose the controls take as the one the turn is applied to, so the arcs are sampled from the
    // term the frame is driven by rather than from a second spelling of it.
    const transform start = body->pose(rigid_motion::rotation_axis_window::frame_object);

    const std::vector<std::shared_ptr<scene::imgui_window>> windows{
            std::make_shared<rigid_motion::rotation_axis_window>("Rotation", *body, motions, drawings_into(*body, motions.screw, start, unit, coordinate), opening())};

    return std::make_shared<scene::preset>(body, windows, site.add_window, site.remove_window);
}

}
