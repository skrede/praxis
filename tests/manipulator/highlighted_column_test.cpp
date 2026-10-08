#include "captured_log.h"
#include "panel_labels.h"
#include "velocity_kinematics_stage.h"

#include "robot/column_arrow.h"

#include "praxis/manipulator/velocity_kinematics_window.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

#include <imgui.h>

#include <threepp/core/Object3D.hpp>

#include <threepp/math/Color.hpp>

#include <threepp/materials/interfaces.hpp>

#include <Eigen/Core>

#include <array>
#include <string>
#include <vector>
#include <cstddef>
#include <optional>
#include <algorithm>

using namespace praxis;
using namespace praxis::tests;
using namespace praxis::fixture;
using namespace praxis::manipulator;

namespace {

using controls  = velocity_kinematics_window::controls;
using opening   = velocity_kinematics_window::settings;
using fill_grid = std::vector<std::vector<std::optional<ImU32>>>;

constexpr const char *panel_title = "Velocity kinematics";
constexpr const char *pick_list   = "Show joint";

constexpr std::size_t matrix_rows = 6u;

constexpr std::array<jacobian_block, 2> both_parts{jacobian_block::angular, jacobian_block::linear};

velocity_kinematics_window opened_over(velocity_stage &headless, const opening &state)
{
    return {panel_title, headless.source->reader(), headless.arm(), headless.shown, controls(), state};
}

// Takes the entry at that place in the list the label names, on a panel walked from its first control.
void take(scene::imgui_window &panel, const char *label, std::size_t entry)
{
    imgui_frame frames;
    const drawing draw = [&panel] { panel.render(); };

    take_entry_on(frames, draw, panel.display_name().c_str(), label, entry);
}

// The tone an arrow standing in the scene wears, taken to the color space the renderer encodes to on
// output, which is the space a panel writes its own colors in.
ImU32 as_written(threepp::Object3D *arrow)
{
    threepp::Object3D *const shaft = shaft_of(arrow);
    REQUIRE(shaft != nullptr);

    threepp::MaterialWithColor *const toned = shaft->materialAs<threepp::MaterialWithColor>();
    REQUIRE(toned != nullptr);
    const unsigned int worn = toned->color.getHex(threepp::SRGBColorSpace);

    return IM_COL32((worn >> 16) & 0xffu, (worn >> 8) & 0xffu, worn & 0xffu, 0xff);
}

// A filled cell's text is black; an unfilled cell carries no tone.
fill_grid fills_carried(const scene::readout &shown)
{
    fill_grid carried;
    for(const std::vector<scene::labeled_value> &row : shown.rows)
    {
        std::vector<std::optional<ImU32>> &fills = carried.emplace_back();
        for(const scene::labeled_value &cell : row)
        {
            if(cell.fill)
                CHECK(cell.tone == IM_COL32_BLACK);
            else
                CHECK_FALSE(cell.tone.has_value());
            fills.push_back(cell.fill);
        }
    }

    return carried;
}

// A reading of that shape with the column marked: its top three cells filled in the tone that column's
// angular arrow wears, its bottom three in the one its linear arrow wears, and no other cell filled.
fill_grid marked(const scene::readout &shown, velocity_stage &headless, std::optional<std::size_t> column)
{
    fill_grid wanted = fills_carried(shown);
    for(std::vector<std::optional<ImU32>> &row : wanted)
        std::fill(row.begin(), row.end(), std::nullopt);
    if(!column)
        return wanted;

    REQUIRE(wanted.size() >= matrix_rows);
    for(std::size_t row = 0; row < matrix_rows; ++row)
    {
        const jacobian_block part = row < 3u ? jacobian_block::angular : jacobian_block::linear;
        wanted[row].at(*column)   = as_written(headless.arrow(*column, part));
    }

    return wanted;
}

// No joint told apart, every arrow of both columns in its part's plain tone, and no cell filled.
void stands_unpicked(velocity_stage &headless, const velocity_kinematics_window &panel)
{
    headless.draw();
    CHECK_FALSE(headless.shown.selected_joint().has_value());
    for(const jacobian_block part : both_parts)
        for(const std::size_t column : {0u, 1u})
            CHECK(arrow_tone(headless.arrow(column, part)) == column_tone(part, false));
    const scene::readout shown = panel.reading();
    CHECK(fills_carried(shown) == marked(shown, headless, std::nullopt));
}

}

TEST_CASE("taking a joint in the list singles its column out in the drawing and fills it in the matrix in its arrows' tones", "[manipulator][window]")
{
    velocity_stage headless;
    headless.put(reading_of(Eigen::Vector3d(1.0, 0.5, 0.25)));
    velocity_kinematics_window panel = opened_over(headless, opening{});
    panel.initialize();

    take(panel, pick_list, 2u);
    headless.draw();

    REQUIRE(headless.shown.selected_joint() == std::optional<std::size_t>(1u));
    for(const jacobian_block part : both_parts)
    {
        CHECK(arrow_tone(headless.arrow(1u, part)) == column_tone(part, true));
        CHECK(arrow_tone(headless.arrow(0u, part)) == column_tone(part, false));
    }

    const scene::readout shown = panel.reading();
    CHECK(fills_carried(shown) == marked(shown, headless, 1u));
    CHECK(as_written(headless.arrow(1u, jacobian_block::angular)) != as_written(headless.arrow(1u, jacobian_block::linear)));
}

TEST_CASE("the list stands directly below the columns switch and directly above the cap switch", "[manipulator][window]")
{
    velocity_stage headless;
    velocity_kinematics_window panel = opened_over(headless, opening{});
    panel.initialize();
    imgui_frame frames;
    const drawing draw = [&panel] { panel.render(); };

    stand_on(frames, draw, panel_title, "Jacobian columns");
    tap(frames, draw, ImGuiKey_DownArrow);
    CHECK(standing_on() == control_id(panel_title, pick_list));
    tap(frames, draw, ImGuiKey_DownArrow);
    CHECK(standing_on() == control_id(panel_title, "Cap the force ellipsoid"));
}

TEST_CASE("taking none after a joint clears the selection, standing every arrow in its plain tone and filling no cell", "[manipulator][window]")
{
    velocity_stage headless;
    headless.put(reading_of(Eigen::Vector3d(1.0, 0.5, 0.25)));
    velocity_kinematics_window panel = opened_over(headless, opening{});
    panel.initialize();

    take(panel, pick_list, 2u);
    REQUIRE(headless.shown.selected_joint() == std::optional<std::size_t>(1u));
    take(panel, pick_list, 0u);

    stands_unpicked(headless, panel);
}

TEST_CASE("a window opened at no pick tells the drawing no joint apart and fills no cell", "[manipulator][window]")
{
    velocity_stage headless;
    headless.put(reading_of(Eigen::Vector3d(1.0, 0.5, 0.25)));
    velocity_kinematics_window panel = opened_over(headless, opening{});
    panel.initialize();

    stands_unpicked(headless, panel);
}

TEST_CASE("the column marked is the one the drawing singles out, however the drawing was told", "[manipulator][window]")
{
    velocity_stage headless;
    headless.put(reading_of(Eigen::Vector3d(1.0, 0.5, 0.25)));
    velocity_kinematics_window panel = opened_over(headless, opening{});
    panel.initialize();

    REQUIRE(headless.shown.set_selected_joint(0u).has_value());
    headless.draw();
    const scene::readout told = panel.reading();
    CHECK(fills_carried(told) == marked(told, headless, 0u));

    headless.shown.clear_selected_joint();
    headless.draw();
    const scene::readout cleared = panel.reading();
    CHECK(fills_carried(cleared) == marked(cleared, headless, std::nullopt));
}

TEST_CASE("the marked column stays marked when the frame moves, carrying the body matrix's numbers", "[manipulator][window]")
{
    velocity_stage headless;
    headless.put(reading_of(Eigen::Vector3d(1.0, 0.5, 0.25)));
    velocity_kinematics_window panel = opened_over(headless, opening{});
    panel.initialize();

    take(panel, pick_list, 1u);
    take(panel, "Jacobian", 1u);
    headless.draw();

    REQUIRE(panel.state().frame == jacobian_frame::body);
    const scene::readout shown = panel.reading();
    CHECK(fills_carried(shown) == marked(shown, headless, 0u));
    const jacobian body = six_by(2u, 100.0);
    for(std::size_t row = 0; row < matrix_rows; ++row)
        CHECK(shown.rows[row][0].value == Catch::Approx(body(static_cast<Eigen::Index>(row), 0)));
}

TEST_CASE("a window opened at a joint tells the drawing that joint apart and marks its column", "[manipulator][window]")
{
    velocity_stage headless;
    headless.put(reading_of(Eigen::Vector3d(1.0, 0.5, 0.25)));
    opening picked{};
    picked.highlighted               = 1u;
    velocity_kinematics_window panel = opened_over(headless, picked);
    panel.initialize();
    headless.draw();

    CHECK(headless.shown.selected_joint() == std::optional<std::size_t>(1u));
    CHECK(panel.state().highlighted == std::optional<std::size_t>(1u));
    const scene::readout shown = panel.reading();
    CHECK(fills_carried(shown) == marked(shown, headless, 1u));
}

TEST_CASE("a window opened at a joint the arm lacks stands at none, the drawing naming the refusal once", "[manipulator][window]")
{
    velocity_stage headless;
    headless.put(reading_of(Eigen::Vector3d(1.0, 0.5, 0.25)));
    opening past{};
    past.highlighted                 = 5u;
    velocity_kinematics_window panel = opened_over(headless, past);

    const std::string reported = reported_by([&panel] { panel.initialize(); });
    const std::size_t first    = reported.find("declined");
    CHECK(first != std::string::npos);
    CHECK(reported.find("declined", first + 1u) == std::string::npos);
    CHECK_FALSE(panel.state().highlighted.has_value());
    stands_unpicked(headless, panel);

    imgui_frame frames;
    frames.draw([&panel] { panel.render(); });
    CHECK(frames.has_draw_data());
}
