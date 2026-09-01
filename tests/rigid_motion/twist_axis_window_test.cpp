#include "panel_keys.h"
#include "imgui_frame.h"

#include "praxis/rigid_motion/axes.h"
#include "praxis/rigid_motion/screw.h"
#include "praxis/rigid_motion/capabilities.h"
#include "praxis/rigid_motion/frame_stencil.h"
#include "praxis/rigid_motion/twist_axis_window.h"

#include "praxis/rigid_motion/baseline/screw.h"

#include "praxis/scene/labeled_value_window.h"

#include "praxis/evaluation/tolerance.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/scenes/Scene.hpp>

#include <Eigen/Core>

#include <vector>
#include <cstddef>
#include <optional>

using namespace praxis;
using namespace praxis::rigid_motion;

namespace {

std::vector<stencil_object> two_objects()
{
    return {stencil_object{"Axis", axes_settings{false}, object_body{body_shape::none, 0.0, nullptr}},
            stencil_object{"Body", axes_settings{}, object_body{body_shape::cube, 0.25, nullptr}}};
}

// The world z-direction through the point (0.5, 0, 0) at a pitch of 0.12 metres per radian, which
// is what the linear part -s x q + h s spells for that point, direction and pitch.
const twist_axis_window::settings opening{Eigen::Vector3f{0.f, 0.f, 1.f}, Eigen::Vector3f{0.f, -0.5f, 0.12f}, 0.75f};

// Distinct from every component the opening twist carries, so a box that took it moved the value
// it held whichever box it was.
constexpr const char *typed_part = "0.4";

// Answers a different axis for every twist the suite passes it, and answers one for a twist the
// unbound implementation would answer nothing useful for.
screw_axis mirrored_axis(const Eigen::Vector3d &w, const Eigen::Vector3d &v)
{
    return screw_axis_from_angular_linear(-w, v);
}

transform turned_further(const screw_axis &s, double theta)
{
    return matrix_exponential_screw(s, theta + 1.0);
}

// The route the stage hands its window: what the window handed it is recorded rather than derived
// again by the case reading it.
twist_axis_window::axis_route recording_into(std::size_t &invocations, std::optional<screw_axis> &named)
{
    return [&invocations, &named](const std::optional<screw_axis> &handed)
    {
        ++invocations;
        named = handed;
    };
}

// One stencil and one window over it, standing where a composition would have left the two.
struct staged
{
    explicit staged(const capabilities &motions)
            : staged(motions, opening)
    {
    }

    staged(const capabilities &motions, const twist_axis_window::settings &chosen)
            : invocations(0)
            , named(std::nullopt)
            , scene()
            , body(scene, two_objects(), motions.frame)
            , panel("Twist", body, motions, recording_into(invocations, named), chosen)
    {
        REQUIRE(body.initialize().has_value());
    }

    staged(const staged &) = delete;

    std::size_t invocations;
    std::optional<screw_axis> named;
    threepp::Scene scene;
    frame_stencil body;
    twist_axis_window panel;
};

fixture::drawing over(twist_axis_window &panel)
{
    return [&panel] { panel.render(); };
}

// Each typing is made in a context of its own, so where the last one left the keyboard cursor
// cannot decide where the next one lands.
void type_component(twist_axis_window &panel, std::size_t below_top, std::size_t along, const char *typed)
{
    tests::imgui_frame frames;
    frames.assert_on_frame_faults(true);

    fixture::type_into_component(frames, over(panel), below_top, along, typed);
}

bool twisted_alike(const twist_axis_window::settings &first, const twist_axis_window::settings &second)
{
    return first.angular_part.cwiseEqual(second.angular_part).all() && first.linear_part.cwiseEqual(second.linear_part).all();
}

// Every control the panel offers is driven in turn and answered for, so which of them move the
// twist comes from the panel rather than from a count of where the panel puts each one. A press
// that moved nothing is as much of the assertion as one that did. A value box takes typed
// characters where a slider takes them only after a different key has opened it, so both
// instruments are offered to every control and one that fitted nothing commits nothing.
std::size_t controls_moving_the_twist(staged &built, tests::imgui_frame &frames, const fixture::drawing &draw, std::size_t offered)
{
    std::size_t moved = 0;
    for(std::size_t step = 0; step < offered; ++step)
    {
        const twist_axis_window::settings before = built.panel.state();
        const std::size_t counted                = built.invocations;

        fixture::type_into(frames, draw, step, typed_part);
        if(twisted_alike(before, built.panel.state()))
            fixture::type_into_slider(frames, draw, step, typed_part);

        const bool changed = !twisted_alike(before, built.panel.state());
        moved += changed ? 1u : 0u;

        CHECK(built.invocations == counted + (changed ? 1u : 0u));
    }

    return moved;
}

std::vector<float> six_of(const scene::readout &shown)
{
    std::vector<float> values;

    REQUIRE(shown.message.empty());
    REQUIRE(shown.rows.size() == 1u);
    for(const scene::labeled_value &cell : shown.rows.front())
    {
        REQUIRE(cell.label.empty());
        values.push_back(cell.value);
    }

    return values;
}

}

TEST_CASE("the_axis_the_controls_derive_is_what_the_bound_operations_answer_for_the_twist")
{
    staged built(baseline());

    built.panel.initialize();

    REQUIRE(built.invocations == 1u);
    REQUIRE(built.named.has_value());
    CHECK(built.named->isApprox(screw_axis_from_angular_linear(opening.angular_part.cast<double>(), opening.linear_part.cast<double>())));
}

TEST_CASE("a_substituted_axis_binding_changes_the_axis_reported_the_reading_and_the_pose_placed")
{
    capabilities substituted                         = baseline();
    substituted.screw.screw_axis_from_angular_linear = &mirrored_axis;

    staged reference(baseline());
    staged perturbed(substituted);

    reference.panel.initialize();
    perturbed.panel.initialize();

    REQUIRE(reference.named.has_value());
    REQUIRE(perturbed.named.has_value());
    CHECK_FALSE(perturbed.named->isApprox(*reference.named));
    CHECK(six_of(perturbed.panel.reading()) != six_of(reference.panel.reading()));
    CHECK_FALSE(is_approx_equal(perturbed.body.pose(twist_axis_window::body_object), reference.body.pose(twist_axis_window::body_object)));

    // The controls place the moving object and nothing else: what stands for the axis is the
    // scenario's, reached through the route.
    CHECK(is_approx_equal(perturbed.body.pose(twist_axis_window::axis_object), transform::Identity()));
    CHECK(is_approx_equal(reference.body.pose(twist_axis_window::axis_object), transform::Identity()));
}

TEST_CASE("the_axis_route_runs_where_the_twist_changed_and_on_no_frame_where_it_did_not")
{
    staged built(baseline());
    built.panel.initialize();

    REQUIRE(built.invocations == 1u);

    tests::imgui_frame frames;
    frames.assert_on_frame_faults(true);
    const fixture::drawing draw = over(built.panel);

    frames.draw(draw, 3);

    CHECK(built.invocations == 1u);

    const std::size_t offered = fixture::navigable_items(frames, draw);

    REQUIRE(offered > 0u);

    const std::size_t moved = controls_moving_the_twist(built, frames, draw, offered);

    // The two rows a twist is typed into, three of its components to a row, and the third control
    // the walk offers is the angle, which places the moving object without asking for the axis
    // again. The normalizing control stands beside the angular row rather than below it, so a walk
    // down the panel never reaches it and it is no part of this count.
    CHECK(moved == 2u);
    CHECK(offered == moved + 1u);
}

TEST_CASE("a_twist_with_neither_an_angular_nor_a_linear_part_names_no_axis_and_moves_nothing")
{
    staged built(baseline(), twist_axis_window::settings{Eigen::Vector3f::Zero(), Eigen::Vector3f::Zero(), opening.angle_radians});
    const transform start = built.body.pose(twist_axis_window::body_object);

    built.panel.initialize();

    REQUIRE(built.invocations == 1u);
    CHECK_FALSE(built.named.has_value());
    CHECK(is_approx_equal(built.body.pose(twist_axis_window::body_object), start));
    CHECK_FALSE(built.panel.reading().message.empty());
    CHECK(built.panel.reading().rows.empty());

    // What the operations answer for that twist is a unit direction like any other, so the guard
    // above is what stands between it and a line drawn along an axis the twist never named.
    CHECK_FALSE(screw_axis_from_angular_linear(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()).isZero());
}

TEST_CASE("a_twist_typed_back_to_zero_gives_up_its_axis_and_leaves_the_moving_object_where_it_was")
{
    staged built(baseline());
    built.panel.initialize();

    // Five of the six, so the last twist that names an axis is a twist standing one component away
    // from naming none and the moving object is where that axis put it. The six stand three to a
    // row, and a row is entered at its leftmost.
    for(std::size_t component = 0; component + 1u < 6u; ++component)
        type_component(built.panel, component / 3u, component % 3u, "0");

    const transform swept = built.body.pose(twist_axis_window::body_object);

    REQUIRE(built.named.has_value());

    type_component(built.panel, 1u, 2u, "0");

    CHECK(built.panel.state().angular_part.isZero());
    CHECK(built.panel.state().linear_part.isZero());
    CHECK_FALSE(built.named.has_value());
    CHECK(is_approx_equal(built.body.pose(twist_axis_window::body_object), swept));
    CHECK_FALSE(built.panel.reading().message.empty());
}

TEST_CASE("the_angle_sweeps_the_moving_object_by_the_bound_exponential_and_leaves_it_at_zero")
{
    capabilities substituted                   = baseline();
    substituted.screw.matrix_exponential_screw = &turned_further;

    staged resting(baseline(), twist_axis_window::settings{opening.angular_part, opening.linear_part, 0.f});
    staged swept(baseline());
    staged driven(substituted);
    const transform start = resting.body.pose(twist_axis_window::body_object);

    resting.panel.initialize();
    swept.panel.initialize();
    driven.panel.initialize();

    REQUIRE(swept.named.has_value());
    CHECK(is_approx_equal(resting.body.pose(twist_axis_window::body_object), start));
    CHECK_FALSE(is_approx_equal(swept.body.pose(twist_axis_window::body_object), start));
    CHECK(is_approx_equal(driven.body.pose(twist_axis_window::body_object), matrix_exponential_screw(*swept.named, opening.angle_radians + 1.0) * start));
}

TEST_CASE("the_reading_is_one_unlabeled_row_of_the_six_components_the_derived_axis_carries")
{
    staged built(baseline());
    staged other(baseline(), twist_axis_window::settings{Eigen::Vector3f{0.f, 1.f, 0.f}, opening.linear_part, opening.angle_radians});

    built.panel.initialize();
    other.panel.initialize();

    const std::vector<float> shown = six_of(built.panel.reading());

    REQUIRE(built.named.has_value());
    REQUIRE(shown.size() == 6u);
    for(std::size_t at = 0; at < shown.size(); ++at)
        CHECK(is_approx_equal(static_cast<double>(shown[at]), (*built.named)[static_cast<Eigen::Index>(at)], 1.0e-6));

    CHECK(six_of(other.panel.reading()) != shown);

    scene::labeled_value_window beside("Screw axis", nullptr, [&built] { return built.panel.reading(); });
    tests::imgui_frame frames;
    frames.assert_on_frame_faults(true);
    frames.draw([&beside] { beside.render(); });

    CHECK(frames.has_draw_data());
    CHECK(frames.vertices() > 0);
}

