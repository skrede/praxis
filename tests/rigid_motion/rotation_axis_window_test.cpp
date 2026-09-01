#include "panel_keys.h"
#include "imgui_frame.h"
#include "captured_log.h"
#include "panel_labels.h"

#include "praxis/rigid_motion/axes.h"
#include "praxis/rigid_motion/screw.h"
#include "praxis/rigid_motion/capabilities.h"
#include "praxis/rigid_motion/frame_stencil.h"
#include "praxis/rigid_motion/rotation_axis_window.h"

#include "praxis/rigid_motion/baseline/frame.h"
#include "praxis/rigid_motion/baseline/screw.h"

#include "praxis/evaluation/tolerance.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/scenes/Scene.hpp>

#include <Eigen/Core>

#include <string>
#include <vector>
#include <cstddef>
#include <optional>

using namespace praxis;
using namespace praxis::rigid_motion;

namespace {

std::vector<stencil_object> one_object()
{
    return {stencil_object{"Frame", axes_settings{}, object_body{body_shape::none, 0.0, nullptr}}};
}

// A direction that is already of unit length and lies along no coordinate axis, so a case reading a
// turn about it reads a turn about a direction the frame carries no arrow along.
const rotation_axis_window::settings opening{Eigen::Vector3f{1.f, 1.f, 1.f}.normalized(), 0.75f, true, true, true, true};

// The direction row is one item of the walk down the panel, the three components of it are stepped
// along that row, and the angle stands below it.
constexpr std::size_t direction_control = 0;
constexpr std::size_t angle_control     = 1;
constexpr std::size_t row_components    = 3;

// Distinct from every component the opening direction carries, so a box that took it moved the value
// it held whichever box it was.
constexpr const char *typed_component = "0.4";

// Answers a different rotation for every axis and angle the suite passes it, and one the unbound
// implementation would not answer for either.
rotation turned_further(const Eigen::Vector3d &w, double theta_radians)
{
    return matrix_exponential_so3(w, theta_radians + 1.0);
}

// The pose an object standing at start is carried to by a turn of that angle about that axis, taken
// through the same two operations the window is bound to.
transform turned(const Eigen::Vector3d &about, double theta_radians, const transform &start)
{
    return transformation_matrix_from_rotation(matrix_exponential_so3(about, theta_radians)) * start;
}

// What the window handed its route, recorded rather than derived again by the case reading it.
struct handed
{
    std::size_t invocations;
    rotation_axis_window::settings shown;
    std::optional<Eigen::Vector3d> named;
};

rotation_axis_window::axis_route recording_into(handed &recorded)
{
    return [&recorded](const rotation_axis_window::settings &shown, const std::optional<Eigen::Vector3d> &named)
    {
        ++recorded.invocations;
        recorded.shown = shown;
        recorded.named = named;
    };
}

// One stencil and one window over it, standing where a composition would have left the two.
struct staged
{
    explicit staged(const capabilities &motions)
            : staged(motions, opening)
    {
    }

    staged(const capabilities &motions, const rotation_axis_window::settings &chosen)
            : recorded{0, chosen, std::nullopt}
            , scene()
            , body(scene, one_object(), motions.frame)
            , panel("Rotation", body, motions, recording_into(recorded), chosen)
    {
        REQUIRE(body.initialize().has_value());
    }

    staged(const staged &) = delete;

    handed recorded;
    threepp::Scene scene;
    frame_stencil body;
    rotation_axis_window panel;
};

fixture::drawing over(rotation_axis_window &panel)
{
    return [&panel] { panel.render(); };
}

const transform &placed(const staged &built)
{
    return built.body.pose(rotation_axis_window::frame_object);
}

// Each typing is made in a context of its own, so where the last one left the keyboard cursor cannot
// decide where the next one lands.
void type_component(rotation_axis_window &panel, std::size_t along, const char *typed)
{
    tests::imgui_frame frames;
    frames.assert_on_frame_faults(true);

    fixture::type_into_component(frames, over(panel), direction_control, along, typed);
}

void drive_angle(rotation_axis_window &panel, const char *typed)
{
    tests::imgui_frame frames;
    frames.assert_on_frame_faults(true);

    fixture::type_into_slider(frames, over(panel), angle_control, typed);
}

// Addressed by the label the panel draws it under rather than by where it stands, so a control the
// panel does not draw fails the case by name.
void press_labeled(rotation_axis_window &panel, const char *label)
{
    tests::imgui_frame frames;
    frames.assert_on_frame_faults(true);

    fixture::press_on(frames, over(panel), panel.display_name().c_str(), label);
}

// The control returning the angle to zero stands beside the angle rather than under it, so a walk
// down the panel never reaches it: it is stepped onto from the row it acts on, and the identifier it
// is then standing on is what says it is that control and not another.
void press_reset(rotation_axis_window &panel)
{
    tests::imgui_frame frames;
    frames.assert_on_frame_faults(true);

    fixture::stand_below_top(frames, over(panel), angle_control);
    fixture::tap(frames, over(panel), ImGuiKey_RightArrow);

    REQUIRE(fixture::standing_on() == fixture::control_id(panel.display_name().c_str(), "Reset"));

    fixture::tap(frames, over(panel), ImGuiKey_Space);
}

// Which flag a switch is supposed to move, so the same four are read after every press and a switch
// reaching past its own is what fails.
struct switched
{
    const char *label;
    bool rotation_axis_window::settings::*held;
};

const std::vector<switched> switches{{"Unit axis", &rotation_axis_window::settings::axis_shown},
                                     {"Coordinate vector", &rotation_axis_window::settings::coordinate_shown},
                                     {"Traversed arc", &rotation_axis_window::settings::arc_shown},
                                     {"Frame", &rotation_axis_window::settings::frame_shown}};

std::size_t occurrences(const std::string &within, const std::string &sought)
{
    std::size_t counted = 0;
    for(std::string::size_type at = within.find(sought); at != std::string::npos; at = within.find(sought, at + sought.size()))
        ++counted;

    return counted;
}

}

TEST_CASE("the_frame_turns_by_the_exponential_the_bound_operations_answer_for_the_axis_and_the_angle")
{
    staged built(baseline());
    const transform start       = placed(built);
    const Eigen::Vector3d about = opening.direction.cast<double>().normalized();

    built.panel.initialize();

    CHECK(is_approx_equal(placed(built), turned(about, static_cast<double>(opening.angle_radians), start), 1.0e-6));

    for(const char *angle : {"1.000", "-2.500", "3.000"})
    {
        INFO(angle);
        drive_angle(built.panel, angle);

        const auto commanded = static_cast<double>(built.panel.state().angle_radians);

        REQUIRE(commanded != static_cast<double>(opening.angle_radians));
        CHECK(is_approx_equal(placed(built), turned(about, commanded, start), 1.0e-5));
    }
}

TEST_CASE("substituting_the_rotation_exponential_alone_puts_the_frame_somewhere_else")
{
    capabilities substituted                 = baseline();
    substituted.screw.matrix_exponential_so3 = &turned_further;

    staged reference(baseline());
    staged perturbed(substituted);
    const transform start       = placed(reference);
    const Eigen::Vector3d about = opening.direction.cast<double>().normalized();

    reference.panel.initialize();
    perturbed.panel.initialize();

    CHECK_FALSE(is_approx_equal(placed(perturbed), placed(reference)));
    CHECK(is_approx_equal(placed(perturbed), turned(about, static_cast<double>(opening.angle_radians) + 1.0, start), 1.0e-6));
}

// The bound operation turns through the length of the vector it is handed times the angle, so a
// window handing it the direction as typed would turn this frame through twice the angle commanded.
TEST_CASE("a_direction_longer_than_one_turns_the_frame_through_the_angle_and_not_a_multiple_of_it")
{
    staged built(baseline(), rotation_axis_window::settings{Eigen::Vector3f{0.f, 0.f, 2.f}, 0.6f, true, true, true, true});
    const transform start = placed(built);
    const auto commanded  = static_cast<double>(built.panel.state().angle_radians);

    built.panel.initialize();

    CHECK(is_approx_equal(placed(built), turned(Eigen::Vector3d::UnitZ(), commanded, start), 1.0e-6));
    CHECK_FALSE(is_approx_equal(placed(built), turned(Eigen::Vector3d{0.0, 0.0, 2.0}, commanded, start), 1.0e-6));
}

TEST_CASE("a_direction_typed_back_to_zero_names_no_axis_and_leaves_the_frame_where_it_was")
{
    staged built(baseline());
    built.panel.initialize();

    // Two of the three, so the last direction that names an axis stands one component away from
    // naming none and the frame is where the turn about that axis put it.
    for(std::size_t along = 0; along + 1u < row_components; ++along)
        type_component(built.panel, along, "0");

    const transform swept = placed(built);

    REQUIRE(built.recorded.named.has_value());

    type_component(built.panel, row_components - 1u, "0");

    CHECK(built.panel.state().direction.isZero());
    CHECK_FALSE(built.recorded.named.has_value());
    CHECK(is_approx_equal(placed(built), swept));
}

// Everything a scenario derives from these controls depends on the angle as well as on the
// direction, so a route that fired on the direction alone would leave all of it stale.
TEST_CASE("the_route_runs_where_only_the_angle_moved_and_is_handed_the_angle_it_moved_to")
{
    staged built(baseline());
    built.panel.initialize();

    REQUIRE(built.recorded.invocations == 1u);

    drive_angle(built.panel, "2.000");

    const float commanded = built.panel.state().angle_radians;

    REQUIRE(commanded != opening.angle_radians);
    CHECK(built.recorded.invocations == 2u);
    CHECK(built.recorded.shown.angle_radians == commanded);
    CHECK(built.recorded.shown.direction.cwiseEqual(opening.direction).all());
    CHECK(built.recorded.named.has_value());
}

TEST_CASE("the_route_is_handed_a_unit_direction_and_nothing_where_the_direction_names_no_axis")
{
    staged built(baseline(), rotation_axis_window::settings{Eigen::Vector3f{0.f, 0.f, 2.f}, opening.angle_radians, true, true, true, true});

    built.panel.initialize();

    REQUIRE(built.recorded.named.has_value());
    CHECK(is_approx_equal(built.recorded.named->norm(), 1.0, 1.0e-9));
    CHECK(is_approx_equal((*built.recorded.named - Eigen::Vector3d::UnitZ()).norm(), 0.0, 1.0e-9));

    for(std::size_t along = 0; along < row_components; ++along)
        type_component(built.panel, along, "0");

    CHECK(built.panel.state().direction.isZero());
    CHECK_FALSE(built.recorded.named.has_value());
}

TEST_CASE("the_panel_offers_the_direction_and_the_angle_and_state_answers_what_the_two_hold")
{
    staged built(baseline());
    built.panel.initialize();

    std::size_t offered = 0;
    {
        tests::imgui_frame frames;
        frames.assert_on_frame_faults(true);
        offered = fixture::navigable_items(frames, over(built.panel));
    }

    // The one row the direction is typed into, its three components stepped along it, the angle
    // below it, and the four switches below that. The normalizing control stands beside the
    // direction row and the control returning the angle to zero stands beside the angle, rather than
    // under the rows they act on, so a walk down the panel never reaches either and neither is any
    // part of this count.
    REQUIRE(offered == 6u);

    const rotation_axis_window::settings before = built.panel.state();

    CHECK(before.direction.cwiseEqual(opening.direction).all());
    CHECK(before.angle_radians == opening.angle_radians);

    type_component(built.panel, 0, typed_component);

    CHECK(is_approx_equal(static_cast<double>(built.panel.state().direction.x()), 0.4, 1.0e-6));
    CHECK(built.panel.state().angle_radians == before.angle_radians);

    drive_angle(built.panel, "2.000");

    CHECK(built.panel.state().angle_radians != before.angle_radians);
    CHECK(is_approx_equal(static_cast<double>(built.panel.state().direction.x()), 0.4, 1.0e-6));
}

// The angle is the one field this control writes, and the window's own change detection compares
// what is shown against what is applied exactly, so a return to zero is read against zero rather
// than against a tolerance.
TEST_CASE("a_control_beside_the_angle_returns_it_to_exactly_zero_and_leaves_the_direction_alone")
{
    staged built(baseline());
    built.panel.initialize();

    drive_angle(built.panel, "2.500");

    const Eigen::Vector3f before = built.panel.state().direction;
    const std::size_t ran        = built.recorded.invocations;

    REQUIRE(built.panel.state().angle_radians != 0.f);

    press_reset(built.panel);

    CHECK(built.panel.state().angle_radians == 0.f);
    CHECK(built.recorded.shown.angle_radians == 0.f);
    CHECK(built.recorded.invocations > ran);

    for(Eigen::Index at = 0; at < 3; ++at)
    {
        INFO(at);
        CHECK(built.panel.state().direction[at] == before[at]);
    }
}

// A control that refuses on every frame it is drawn refuses silently by repetition, so the refusal
// is counted over a run of frames rather than read out of one.
TEST_CASE("a_direction_naming_no_axis_is_refused_once_however_many_frames_are_drawn")
{
    staged built(baseline(), rotation_axis_window::settings{Eigen::Vector3f::Zero(), opening.angle_radians, true, true, true, true});
    const transform start = placed(built);

    const std::string reported = tests::reported_by(
            [&built]
            {
                built.panel.initialize();

                tests::imgui_frame frames;
                frames.assert_on_frame_faults(true);
                frames.draw(over(built.panel), 4);
            });

    CHECK(occurrences(reported, "names no axis") == 1u);
    CHECK_FALSE(built.recorded.named.has_value());
    CHECK(is_approx_equal(placed(built), start));
}

// The drawings these controls do not own are rebuilt through the route, so a switch that does not
// reach it is a switch that hides nothing.
TEST_CASE("the_route_is_handed_the_flags_the_switches_hold")
{
    staged built(baseline());
    built.panel.initialize();

    REQUIRE(built.recorded.invocations == 1u);
    REQUIRE(built.recorded.shown.axis_shown);
    REQUIRE(built.recorded.shown.coordinate_shown);
    REQUIRE(built.recorded.shown.arc_shown);
    REQUIRE(built.recorded.shown.frame_shown);

    for(const switched &pressed : switches)
    {
        INFO(pressed.label);

        const std::size_t before = built.recorded.invocations;

        press_labeled(built.panel, pressed.label);

        CHECK(built.recorded.invocations > before);
        CHECK_FALSE(built.panel.state().*pressed.held);

        for(const switched &subject : switches)
        {
            INFO(subject.label);
            CHECK(built.recorded.shown.*subject.held == (subject.held != pressed.held));
        }

        press_labeled(built.panel, pressed.label);

        CHECK(built.recorded.shown.*pressed.held);
    }
}
