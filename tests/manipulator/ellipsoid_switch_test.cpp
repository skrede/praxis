#include "panel_labels.h"
#include "velocity_kinematics_stage.h"

#include "../presets/drawn_lines.h"

#include "praxis/manipulator/velocity_kinematics_window.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/core/Object3D.hpp>

#include <Eigen/Core>

#include <string>

using namespace praxis;
using namespace praxis::tests;
using namespace praxis::fixture;
using namespace praxis::manipulator;

namespace {

using controls = velocity_kinematics_window::controls;
using opening  = velocity_kinematics_window::settings;

constexpr const char *panel_title = "Velocity kinematics";

// The names the panel writes beside the two switches, which are what a reader has to go on.
constexpr const char *angular_switch = "Angular ellipsoid";
constexpr const char *linear_switch  = "Linear ellipsoid";

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

// Both bodies stand at the opening, then the named switch is pressed and only the body it names has
// moved.
void moves_only(velocity_stage &headless, const char *label, jacobian_block named, jacobian_block other)
{
    velocity_kinematics_window panel = opened_over(headless, opening{});
    panel.initialize();
    headless.draw();
    REQUIRE(drawn(headless.body(named)));
    REQUIRE(drawn(headless.body(other)));

    flip(panel, label);
    headless.draw();

    CHECK_FALSE(drawn(headless.body(named)));
    CHECK(drawn(headless.body(other)));
}

// Drives a panel opened with both switches at one value to the combination asked for, pressing only
// the switches that have to move, and reads back which bodies the stencil then draws.
void reaches(velocity_stage &headless, bool from, bool angular_on, bool linear_on)
{
    opening opened{};
    opened.angular_ellipsoid = from;
    opened.linear_ellipsoid  = from;

    velocity_kinematics_window panel = opened_over(headless, opened);
    panel.initialize();
    if(angular_on != from)
        flip(panel, angular_switch);
    if(linear_on != from)
        flip(panel, linear_switch);
    headless.draw();

    CHECK(drawn(headless.body(jacobian_block::angular)) == angular_on);
    CHECK(drawn(headless.body(jacobian_block::linear)) == linear_on);
}

}

TEST_CASE("the panel offers a switch under each ellipsoid's own name", "[manipulator][window]")
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

TEST_CASE("the switch named for the angular ellipsoid takes that body away and leaves the linear one drawn", "[manipulator][window]")
{
    velocity_stage headless;
    headless.put(reading_of(readable()));

    moves_only(headless, angular_switch, jacobian_block::angular, jacobian_block::linear);
}

TEST_CASE("the switch named for the linear ellipsoid takes that body away and leaves the angular one drawn", "[manipulator][window]")
{
    velocity_stage headless;
    headless.put(reading_of(readable()));

    moves_only(headless, linear_switch, jacobian_block::linear, jacobian_block::angular);
}

TEST_CASE("each combination of the two named switches draws exactly the bodies those switches stand for", "[manipulator][window]")
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
