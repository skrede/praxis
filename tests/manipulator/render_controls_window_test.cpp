#include "velocity_kinematics_stage.h"

#include "praxis/manipulator/render_controls_window.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <imgui.h>

#include <set>
#include <array>
#include <string>
#include <cstddef>
#include <optional>
#include <string_view>

using namespace praxis;
using namespace praxis::tests;
using namespace praxis::fixture;
using namespace praxis::manipulator;

namespace {

using controls = render_controls_window::controls;
using opening  = render_controls_window::settings;

constexpr const char *panel_title    = "Render controls";
constexpr std::string_view render_at = "machine/render_controls";

// The rows the five fields stand on, counted from the panel's first, with every control offered.
constexpr int angular_row        = 0;
constexpr int linear_row         = 1;
constexpr int cap_row            = 2;
constexpr int angular_column_row = 3;
constexpr int linear_column_row  = 4;

// Outside every widget's own increment and equal to none of the extents the stage told the stencil,
// so a value found at one of these was typed into that field.
constexpr const char *typed_text = "0.125";
constexpr double typed           = 0.125;

// The five lengths, in the order the panel draws them, which is the order the rows above count in.
using five_lengths = std::array<double, 5u>;

five_lengths standing(const loadable_robot_stencil &shown)
{
    return {shown.ellipsoid_scale(jacobian_block::angular), shown.ellipsoid_scale(jacobian_block::linear), shown.force_cap_ratio(), shown.column_scale(jacobian_block::angular),
            shown.column_scale(jacobian_block::linear)};
}

five_lengths offered_by(const opening &state)
{
    return {state.angular_scale.value_or(0.0), state.linear_scale.value_or(0.0), state.force_cap_ratio.value_or(0.0), state.angular_column_scale.value_or(0.0),
            state.linear_column_scale.value_or(0.0)};
}

opening every_length_named()
{
    return opening{0.05, 0.06, 0.07, 0.08, 1.5};
}

controls offering(int subset)
{
    controls offered;
    offered.ellipsoid_scale = (subset & 1) != 0;
    offered.force_cap       = (subset & 2) != 0;
    offered.column_scale    = (subset & 4) != 0;

    return offered;
}

std::size_t bare_panel()
{
    return geometry_of(
            []
            {
                ImGui::Begin(panel_title);
                ImGui::End();
            });
}

// Each walk starts from the top of a freshly focused panel, so a case reads the order the panel
// draws in rather than wherever the walk before it left the cursor.
void type_into(scene::imgui_window &panel, int row, const char *text)
{
    imgui_frame frames;
    const drawing draw = over(panel);
    start_navigating(frames, draw);
    reach(frames, draw, ImGuiKey_Home);
    for(int step = 0; step < row; ++step)
        tap(frames, draw, ImGuiKey_DownArrow);

    type_at_cursor(frames, draw, text);
}

void each_length_at(const five_lengths &read, const five_lengths &expected)
{
    for(std::size_t which = 0; which < read.size(); ++which)
    {
        INFO(which);
        CHECK(read[which] == Catch::Approx(expected[which]));
    }
}

}

TEST_CASE("a window draws the fields its composition asked for and no others", "[manipulator][controls]")
{
    velocity_stage headless;

    std::set<std::size_t> panels;
    for(int subset = 0; subset < 8; ++subset)
    {
        render_controls_window panel(panel_title, headless.shown, offering(subset), opening{});
        panels.insert(geometry_of(over(panel)));
    }

    CHECK(panels.size() == 8u);

    render_controls_window offering_nothing(panel_title, headless.shown, offering(0), opening{});

    CHECK(geometry_of(over(offering_nothing)) == bare_panel());
}

// Every length a composition named reaches the stencil whether or not a control was drawn for it,
// which is what leaves a length nobody offered a control for standing where the composition put it.
TEST_CASE("every length a composition named reaches the stencil and is offered back, with no control drawn for any of them", "[manipulator][controls]")
{
    velocity_stage headless;
    render_controls_window panel(panel_title, headless.shown, offering(0), every_length_named(), std::string(render_at));
    panel.initialize();

    each_length_at(standing(headless.shown), offered_by(every_length_named()));
    each_length_at(offered_by(panel.state()), offered_by(every_length_named()));
}

// A length nobody named is the stencil's own, proportioned to whichever arm is rendered. Writing it
// back as an explicit value would fix one machine's proportion into the document and hand it to
// every other, so an unmoved length is offered as absence.
TEST_CASE("a length the composition left unnamed opens at the extent the stencil carries and is offered back as absence", "[manipulator][controls]")
{
    velocity_stage headless;
    const five_lengths opened = standing(headless.shown);

    render_controls_window panel(panel_title, headless.shown, controls(), opening{}, std::string(render_at));
    panel.initialize();
    each_length_at(standing(headless.shown), opened);

    const opening answered = panel.state();
    CHECK_FALSE(answered.angular_scale.has_value());
    CHECK_FALSE(answered.linear_scale.has_value());
    CHECK_FALSE(answered.angular_column_scale.has_value());
    CHECK_FALSE(answered.linear_column_scale.has_value());
    CHECK_FALSE(answered.force_cap_ratio.has_value());
}

// A drawing of no extent draws nothing while still reporting a length, so no field admits one: what
// is typed at or below zero stands at the smallest extent its own kind draws at, a length for the
// four scales and a multiple for the cut.
TEST_CASE("each field writes its own length, reaches none of the four beside it, and admits no extent that would draw nothing", "[manipulator][controls]")
{
    for(const int row : {angular_row, linear_row, cap_row, angular_column_row, linear_column_row})
    {
        INFO("row " << row);
        velocity_stage headless;
        const five_lengths opened = standing(headless.shown);
        const std::size_t moved   = static_cast<std::size_t>(row);

        render_controls_window panel(panel_title, headless.shown, controls(), opening{});
        panel.initialize();
        type_into(panel, row, typed_text);

        five_lengths expected = opened;
        expected[moved]       = typed;
        each_length_at(standing(headless.shown), expected);

        type_into(panel, row, "-1");

        CHECK(standing(headless.shown)[moved] > 0.0);
        CHECK(offered_by(panel.state())[moved] > 0.0);
    }
}

TEST_CASE("a window no key path was named for offers nothing to write, and one named a path answers for itself", "[manipulator][configuration]")
{
    velocity_stage headless;
    render_controls_window unrouted(panel_title, headless.shown);

    CHECK(unrouted.settings_path().empty());
    CHECK(unrouted.as_configurable() == nullptr);

    render_controls_window routed(panel_title, headless.shown, controls(), opening{}, std::string(render_at));

    CHECK(routed.settings_path() == render_at);
    CHECK(routed.as_configurable() == &routed);
}
