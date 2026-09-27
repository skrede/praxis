#include "panel_labels.h"
#include "velocity_kinematics_stage.h"

#include "../presets/drawn_lines.h"

#include "praxis/manipulator/velocity_kinematics_window.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/core/Object3D.hpp>

#include <imgui.h>

#include <Eigen/Core>

#include <array>
#include <string>
#include <vector>
#include <cstddef>

using namespace praxis;
using namespace praxis::tests;
using namespace praxis::fixture;
using namespace praxis::manipulator;

namespace {

using controls = velocity_kinematics_window::controls;
using opening  = velocity_kinematics_window::settings;

constexpr const char *panel_title = "Velocity kinematics";

// The names the panel writes beside the two switches, which are what a reader has to go on.
constexpr const char *angular_switch    = "Angular";
constexpr const char *linear_switch     = "Linear";
constexpr const char *ellipsoids_switch = "Ellipsoids";
constexpr const char *columns_switch    = "Jacobian columns";

// Singular values a decomposition can be read off, all three positive, so both bodies have an extent
// to draw and neither is refused.
inline Eigen::Vector3d readable()
{
    return {1.0, 0.5, 0.25};
}

velocity_kinematics_window opened_over(velocity_stage &headless, const opening &state)
{
    return {panel_title, headless.source->reader(), headless.arm(), headless.shown, controls(), state};
}

// Presses the switch the label names, on a panel freshly focused and walked from its first control.
// The walk is what carries the name: a label the panel does not draw ends the case by that name
// rather than pressing whatever control now stands where it used to.
void flip(scene::imgui_window &panel, const char *label)
{
    imgui_frame frames;
    const drawing draw = [&panel] { panel.render(); };

    press_on(frames, draw, panel.display_name().c_str(), label);
}

// Both drawings of both parts stand at the opening, then the named switch is pressed and only the
// part it names has moved -- its body and its arrow of every column together.
void moves_only(velocity_stage &headless, const char *label, jacobian_block named, jacobian_block other)
{
    velocity_kinematics_window panel = opened_over(headless, opening{});
    panel.initialize();
    headless.draw();
    REQUIRE(drawn(headless.body(named)));
    REQUIRE(drawn(headless.body(other)));
    REQUIRE(drawn(headless.arrow(0u, named)));
    REQUIRE(drawn(headless.arrow(0u, other)));

    flip(panel, label);
    headless.draw();

    CHECK_FALSE(drawn(headless.body(named)));
    CHECK(drawn(headless.body(other)));
    CHECK_FALSE(drawn(headless.arrow(0u, named)));
    CHECK_FALSE(drawn(headless.arrow(1u, named)));
    CHECK(drawn(headless.arrow(0u, other)));
}

// Drives a panel opened with both switches at one value to the combination asked for, pressing only
// the switches that have to move, and reads back which bodies the stencil then draws.
void reaches(velocity_stage &headless, bool from, bool angular_on, bool linear_on)
{
    opening opened{};
    opened.angular = from;
    opened.linear  = from;

    velocity_kinematics_window panel = opened_over(headless, opened);
    panel.initialize();
    if(angular_on != from)
        flip(panel, angular_switch);
    if(linear_on != from)
        flip(panel, linear_switch);
    headless.draw();

    CHECK(drawn(headless.body(jacobian_block::angular)) == angular_on);
    CHECK(drawn(headless.body(jacobian_block::linear)) == linear_on);
    CHECK(drawn(headless.arrow(0u, jacobian_block::angular)) == angular_on);
    CHECK(drawn(headless.arrow(1u, jacobian_block::linear)) == linear_on);
}

constexpr std::array<jacobian_block, 2> both_parts{jacobian_block::angular, jacobian_block::linear};

// Whether each part's body and each of its continuation lines is drawn, part by part.
std::vector<bool> ellipsoids_drawn(velocity_stage &headless)
{
    std::vector<bool> standing;
    for(const jacobian_block part : both_parts)
    {
        standing.push_back(drawn(headless.body(part)));
        for(std::size_t axis = 0u; axis < 3u; ++axis)
            for(const bool forward : {true, false})
                standing.push_back(drawn(headless.line(part, axis, forward)));
    }

    return standing;
}

bool every_arrow_drawn(velocity_stage &headless)
{
    bool standing = true;
    for(const jacobian_block part : both_parts)
        for(std::size_t column = 0u; column < 2u; ++column)
            standing = standing && drawn(headless.arrow(column, part));

    return standing;
}

// The three switches opened at one value and driven to the combination asked for, the part switches
// pressed before the switch over both ellipsoids, so a part moved while the ellipsoids were hidden is
// what turning them back on has to honor.
void reaches_under_ellipsoids(velocity_stage &headless, bool from, bool ellipsoids_on, bool angular_on, bool linear_on)
{
    opening opened{};
    opened.angular    = from;
    opened.linear     = from;
    opened.ellipsoids = from;

    velocity_kinematics_window panel = opened_over(headless, opened);
    panel.initialize();
    if(angular_on != from)
        flip(panel, angular_switch);
    if(linear_on != from)
        flip(panel, linear_switch);
    if(ellipsoids_on != from)
        flip(panel, ellipsoids_switch);
    headless.draw();

    CHECK(drawn(headless.body(jacobian_block::angular)) == (ellipsoids_on && angular_on));
    CHECK(drawn(headless.body(jacobian_block::linear)) == (ellipsoids_on && linear_on));
    CHECK(drawn(headless.arrow(1u, jacobian_block::angular)) == angular_on);
    CHECK(drawn(headless.arrow(0u, jacobian_block::linear)) == linear_on);
}

}

TEST_CASE("the panel offers a switch under each part's own name", "[manipulator][window]")
{
    velocity_stage headless;
    headless.put(reading_of(readable()));

    velocity_kinematics_window panel = opened_over(headless, opening{});
    panel.initialize();

    imgui_frame frames;
    const drawing draw       = [&panel] { panel.render(); };
    const std::string titled = panel.display_name();

    stand_on(frames, draw, titled.c_str(), angular_switch);
    stand_on(frames, draw, titled.c_str(), linear_switch);
}

TEST_CASE("the switch named for the angular part takes that body and that part's arrows away and leaves the linear ones drawn", "[manipulator][window]")
{
    velocity_stage headless;
    headless.put(reading_of(readable()));

    moves_only(headless, angular_switch, jacobian_block::angular, jacobian_block::linear);
}

TEST_CASE("the switch named for the linear part takes that body and that part's arrows away and leaves the angular ones drawn", "[manipulator][window]")
{
    velocity_stage headless;
    headless.put(reading_of(readable()));

    moves_only(headless, linear_switch, jacobian_block::linear, jacobian_block::angular);
}

TEST_CASE("each combination of the two named switches draws exactly the bodies and the arrows those switches stand for", "[manipulator][window]")
{
    velocity_stage headless;
    headless.put(reading_of(readable()));

    for(const bool from : {true, false})
        for(const bool angular_on : {true, false})
            for(const bool linear_on : {true, false})
            {
                INFO("opened at " << from << ", driven to angular " << angular_on << " and linear " << linear_on);
                reaches(headless, from, angular_on, linear_on);
            }
}

TEST_CASE("the panel offers a switch over both ellipsoids directly above the columns switch", "[manipulator][window]")
{
    velocity_stage headless;
    headless.put(reading_of(readable()));

    velocity_kinematics_window panel = opened_over(headless, opening{});
    panel.initialize();

    imgui_frame frames;
    const drawing draw       = [&panel] { panel.render(); };
    const std::string titled = panel.display_name();

    stand_on(frames, draw, titled.c_str(), ellipsoids_switch);
    tap(frames, draw, ImGuiKey_DownArrow);
    CHECK(standing_on() == control_id(titled.c_str(), columns_switch));
}

// Opened at the force reading, under which the continuation lines of these bodies are drawn.
TEST_CASE("the switch over both ellipsoids hides both bodies and their lines, moves nothing else, and puts back what it hid", "[manipulator][window]")
{
    velocity_stage headless;
    headless.put(reading_of(readable()));
    opening opened{};
    opened.reading                   = ellipsoid_view::force;
    velocity_kinematics_window panel = opened_over(headless, opened);
    panel.initialize();
    headless.draw();
    const std::vector<bool> before = ellipsoids_drawn(headless);
    const opening held             = panel.state();
    REQUIRE(before == std::vector<bool>(before.size(), true));

    flip(panel, ellipsoids_switch);
    headless.draw();
    CHECK(ellipsoids_drawn(headless) == std::vector<bool>(before.size(), false));
    CHECK(every_arrow_drawn(headless));
    CHECK(panel.state().angular == held.angular);
    CHECK(panel.state().linear == held.linear);
    CHECK(panel.state().columns == held.columns);
    CHECK(panel.state().reading == held.reading);

    flip(panel, ellipsoids_switch);
    headless.draw();
    CHECK(ellipsoids_drawn(headless) == before);
}

TEST_CASE("each combination of the switch over both ellipsoids and the two part switches draws the bodies both allow and the arrows the parts allow", "[manipulator][window]")
{
    velocity_stage headless;
    headless.put(reading_of(readable()));

    for(const bool from : {true, false})
        for(const bool ellipsoids_on : {true, false})
            for(const bool angular_on : {true, false})
                for(const bool linear_on : {true, false})
                {
                    INFO("opened at " << from << ", driven to ellipsoids " << ellipsoids_on << ", angular " << angular_on << " and linear " << linear_on);
                    reaches_under_ellipsoids(headless, from, ellipsoids_on, angular_on, linear_on);
                }
}

TEST_CASE("a window opened with both ellipsoids hidden draws neither body, whatever reading the stencil is then told", "[manipulator][window]")
{
    velocity_stage headless;
    headless.put(reading_of(readable()));

    opening opened{};
    opened.ellipsoids                = false;
    velocity_kinematics_window panel = opened_over(headless, opened);
    panel.initialize();
    headless.draw();
    CHECK_FALSE(drawn(headless.body(jacobian_block::angular)));
    CHECK_FALSE(drawn(headless.body(jacobian_block::linear)));
    CHECK(every_arrow_drawn(headless));

    headless.shown.set_manipulability_ellipsoids(ellipsoid_view::force);
    headless.draw();
    CHECK_FALSE(drawn(headless.body(jacobian_block::angular)));
    CHECK_FALSE(drawn(headless.body(jacobian_block::linear)));
}
