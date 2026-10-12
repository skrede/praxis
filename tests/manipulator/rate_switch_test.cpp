#include "panel_labels.h"
#include "velocity_kinematics_stage.h"

#include "../presets/drawn_lines.h"

#include "praxis/manipulator/velocity_kinematics_window.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <threepp/core/Object3D.hpp>

#include <imgui.h>

#include <Eigen/Core>

#include <vector>
#include <cstddef>
#include <optional>

using namespace praxis;
using namespace praxis::tests;
using namespace praxis::fixture;
using namespace praxis::manipulator;

namespace {

using controls = velocity_kinematics_window::controls;
using opening  = velocity_kinematics_window::settings;

constexpr const char *panel_title      = "Velocity kinematics";
constexpr const char *cap_switch       = "Cap the force ellipsoid";
constexpr const char *weighting_switch = "Columns times joint rates";
constexpr const char *twist_switch     = "Twist J times rates";

constexpr double head_length = 0.02;

constexpr jacobian_block both_parts[] = {jacobian_block::angular, jacobian_block::linear};

const std::vector<double> told_rates{2.0, 1.0};

velocity_kinematics_window opened_over(velocity_stage &headless, const controls &offered, const opening &state)
{
    return {panel_title, headless.source->reader(), headless.arm(), headless.shown, offered, state};
}

opening carrying(std::optional<bool> rate_weighted, std::optional<bool> twist)
{
    opening state;
    state.rate_weighted = rate_weighted;
    state.twist         = twist;

    return state;
}

// A reading published and the rates told, so either switch has something to draw.
void told(velocity_stage &headless)
{
    headless.put(reading_of(Eigen::Vector3d(1.0, 0.5, 0.25)));
    REQUIRE(headless.shown.set_joint_rates(told_rates).has_value());
}

// Presses the switch the label names, on a panel freshly focused and walked from its first control,
// so a label the panel does not draw ends the case by that name.
void flip(scene::imgui_window &panel, const char *label)
{
    imgui_frame frames;
    const drawing draw = [&panel] { panel.render(); };

    press_on(frames, draw, panel.display_name().c_str(), label);
}

// The shaft and the head end to end.
double drawn_length(threepp::Object3D *arrow)
{
    return shaft_stretch(arrow) + head_length;
}

threepp::Object3D *twist_arrow(velocity_stage &headless, jacobian_block part)
{
    return headless.scene->getObjectByName<threepp::Object3D>(loadable_robot_stencil::twist_name(part));
}

// From the cap switch the walk down stands on each label in turn, and a step past the last leaves the
// cursor where it was.
void stands_below_the_cap(scene::imgui_window &panel, const std::vector<const char *> &labels)
{
    imgui_frame frames;
    const drawing draw = [&panel] { panel.render(); };

    stand_on(frames, draw, panel_title, cap_switch);
    for(const char *label : labels)
    {
        tap(frames, draw, ImGuiKey_DownArrow);
        CHECK(standing_on() == control_id(panel_title, label));
    }

    const ImGuiID last = standing_on();
    tap(frames, draw, ImGuiKey_DownArrow);
    CHECK(standing_on() == last);
}

bool one_below_the_cap(scene::imgui_window &panel, const char *label)
{
    imgui_frame frames;
    const drawing draw = [&panel] { panel.render(); };

    stand_on(frames, draw, panel_title, cap_switch);
    tap(frames, draw, ImGuiKey_DownArrow);

    return standing_on() == control_id(panel_title, label);
}

std::size_t signature_of(scene::imgui_window &panel)
{
    imgui_frame frames;
    frames.draw([&panel] { panel.render(); });

    return frames.signature();
}

}

TEST_CASE("an opted-in window stands the weighting switch directly after the cap switch, and pressing it weights the columns of told rates", "[manipulator][window]")
{
    velocity_stage headless;
    told(headless);
    velocity_kinematics_window panel = opened_over(headless, controls(), carrying(false, std::nullopt));
    panel.initialize();
    headless.draw();
    const double unit = drawn_length(headless.arrow(0u, jacobian_block::angular));
    REQUIRE(one_below_the_cap(panel, weighting_switch));

    flip(panel, weighting_switch);
    headless.draw();
    CHECK(panel.state().rate_weighted == std::optional<bool>(true));
    CHECK(headless.shown.columns_weighted());
    CHECK(drawn_length(headless.arrow(0u, jacobian_block::angular)) == Catch::Approx(2.0 * unit).margin(1e-6));

    flip(panel, weighting_switch);
    headless.draw();
    CHECK(panel.state().rate_weighted == std::optional<bool>(false));
    CHECK(drawn_length(headless.arrow(0u, jacobian_block::angular)) == Catch::Approx(unit).margin(1e-6));
}

TEST_CASE("the two rate switches stand after the cap switch in one order, the weighting first", "[manipulator][window]")
{
    velocity_stage headless;
    velocity_kinematics_window panel = opened_over(headless, controls(), carrying(false, false));

    stands_below_the_cap(panel, {weighting_switch, twist_switch});
}

TEST_CASE("a window over settings carrying only the weighting draws only the weighting switch after the cap switch", "[manipulator][window]")
{
    velocity_stage headless;
    velocity_kinematics_window panel = opened_over(headless, controls(), carrying(true, std::nullopt));

    stands_below_the_cap(panel, {weighting_switch});
}

TEST_CASE("a window over settings carrying neither rate switch draws nothing after the cap switch", "[manipulator][window]")
{
    velocity_stage headless;
    velocity_kinematics_window panel = opened_over(headless, controls(), opening{});

    stands_below_the_cap(panel, {});
}

TEST_CASE("pressing the twist switch draws and withdraws the twist of the told rates", "[manipulator][window]")
{
    velocity_stage headless;
    told(headless);
    velocity_kinematics_window panel = opened_over(headless, controls(), carrying(std::nullopt, false));
    panel.initialize();
    headless.draw();
    for(const jacobian_block part : both_parts)
        REQUIRE_FALSE(drawn(twist_arrow(headless, part)));

    flip(panel, twist_switch);
    headless.draw();
    CHECK(panel.state().twist == std::optional<bool>(true));
    CHECK(headless.shown.twist_shown());
    for(const jacobian_block part : both_parts)
        CHECK(drawn(twist_arrow(headless, part)));

    flip(panel, twist_switch);
    headless.draw();
    CHECK(panel.state().twist == std::optional<bool>(false));
    CHECK_FALSE(headless.shown.twist_shown());
    for(const jacobian_block part : both_parts)
        CHECK_FALSE(drawn(twist_arrow(headless, part)));
}

TEST_CASE("a window offering no switches draws neither rate switch but still tells the stencil", "[manipulator][window]")
{
    velocity_stage headless;
    told(headless);
    controls withheld;
    withheld.shown = false;

    velocity_kinematics_window offered = opened_over(headless, controls(), carrying(true, true));
    velocity_kinematics_window unopted = opened_over(headless, controls(), opening{});
    REQUIRE(signature_of(offered) != signature_of(unopted));

    velocity_kinematics_window bare  = opened_over(headless, withheld, opening{});
    velocity_kinematics_window panel = opened_over(headless, withheld, carrying(true, true));
    panel.initialize();

    CHECK(signature_of(panel) == signature_of(bare));
    CHECK(headless.shown.columns_weighted());
    CHECK(headless.shown.twist_shown());
}
