#include "captured_log.h"
#include "velocity_kinematics_stage.h"

#include "../presets/drawn_lines.h"

#include "praxis/manipulator/velocity_kinematics_window.h"

#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <imgui.h>

#include <threepp/core/Object3D.hpp>

#include <threepp/math/Color.hpp>

#include <threepp/materials/interfaces.hpp>

#include <Eigen/Core>

#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <limits>
#include <cstddef>
#include <algorithm>
#include <string_view>

using namespace praxis;
using namespace praxis::tests;
using namespace praxis::fixture;
using namespace praxis::manipulator;
using Catch::Matchers::ContainsSubstring;

namespace {

using controls = velocity_kinematics_window::controls;
using opening  = velocity_kinematics_window::settings;

constexpr const char *panel_title      = "Velocity kinematics";
constexpr std::string_view velocity_at = "machine/velocity";

// The rows a control stands on, counted from the panel's first, with every control offered.
constexpr int jacobian_row = 0;
constexpr int reading_row  = 1;
constexpr int angular_row  = 2;
constexpr int linear_row   = 3;
constexpr int columns_row  = 4;
constexpr int capped_row   = 5;

controls no_control()
{
    controls offered;
    offered.frame   = false;
    offered.reading = false;
    offered.shown   = false;

    return offered;
}

// The panel's first six rows are the matrix, whatever the joint count, so the block rows are what
// stands after them: a values row and a scalars row for a block that decomposed, one row for a block
// that refused.
constexpr std::size_t matrix_rows = 6u;

const std::vector<scene::labeled_value> &after_matrix(const scene::readout &shown, std::size_t row)
{
    REQUIRE(shown.rows.size() > matrix_rows + row);

    return shown.rows[matrix_rows + row];
}

void step_to(imgui_frame &frames, const drawing &draw, int row)
{
    reach(frames, draw, ImGuiKey_Home);
    for(int step = 0; step < row; ++step)
        tap(frames, draw, ImGuiKey_DownArrow);
}

// Each walk starts from the top of a freshly focused panel, so a case reads the order the panel
// draws in rather than wherever the walk before it left the cursor.
void from_the_top(scene::imgui_window &panel, imgui_frame &frames, const drawing &draw, int row)
{
    ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    frames.draw(
            [&panel]
            {
                ImGui::SetNextWindowFocus();
                panel.render();
            });

    step_to(frames, draw, row);
}

// The list a cycle draws opens its keyboard cursor on its first entry rather than on the entry it is
// showing, so the count is a position in the list.
void choose_entry(scene::imgui_window &panel, int row, int entry)
{
    imgui_frame frames;
    const drawing draw = [&panel] { panel.render(); };

    from_the_top(panel, frames, draw, row);
    tap(frames, draw, ImGuiKey_Space);
    for(int step = 0; step < entry; ++step)
        tap(frames, draw, ImGuiKey_DownArrow);

    tap(frames, draw, ImGuiKey_Space);
}

void press(scene::imgui_window &panel, int row)
{
    imgui_frame frames;
    const drawing draw = [&panel] { panel.render(); };

    from_the_top(panel, frames, draw, row);
    tap(frames, draw, ImGuiKey_Space);
}

// Every colour the panel put on screen, so a case reads the tones a drawing carries rather than the
// count of what it drew: a vertex costs the same whatever colour it wears.
std::vector<ImU32> tones_drawn(scene::imgui_window &panel)
{
    imgui_frame frames;
    frames.draw([&panel] { panel.render(); });

    const ImDrawData *shown = ImGui::GetDrawData();
    REQUIRE(shown != nullptr);

    std::vector<ImU32> seen;
    for(int list = 0; list < shown->CmdListsCount; ++list)
        for(const ImDrawVert &vertex : shown->CmdLists[list]->VtxBuffer)
            seen.push_back(vertex.col);

    return seen;
}

bool carries(const std::vector<ImU32> &seen, ImU32 tone)
{
    return std::find(seen.begin(), seen.end(), tone) != seen.end();
}

// The tone an arrow standing in the scene wears, taken to the colour space the renderer encodes to on
// output, which is the space a panel writes its own colours in.
ImU32 as_written(threepp::Object3D *arrow)
{
    threepp::Object3D *const shaft = shaft_of(arrow);
    REQUIRE(shaft != nullptr);

    threepp::MaterialWithColor *const toned = shaft->materialAs<threepp::MaterialWithColor>();
    REQUIRE(toned != nullptr);
    const unsigned int worn = toned->color.getHex(threepp::SRGBColorSpace);

    return IM_COL32((worn >> 16) & 0xffu, (worn >> 8) & 0xffu, worn & 0xffu, 0xff);
}

std::size_t signature_of(scene::imgui_window &panel)
{
    imgui_frame frames;
    frames.draw([&panel] { panel.render(); });

    return frames.signature();
}

}

TEST_CASE("a window over an arm that has published nothing answers a message and no rows", "[manipulator][window]")
{
    velocity_stage headless;
    velocity_kinematics_window panel(panel_title, headless.source->reader(), headless.arm(), headless.shown);

    CHECK(panel.reading().message == std::string(absent_line));
    CHECK(panel.reading().rows.empty());
}

TEST_CASE("a chosen Jacobian that is a refusal is named, and the other frame still reads its matrix", "[manipulator][window]")
{
    velocity_stage headless;
    headless.put(reading_of(unexpected(refusal::not_implemented), six_by(2u, 100.0), refused(), decomposed_both(Eigen::Vector3d(1.0, 0.5, 0.25))));

    velocity_kinematics_window space(panel_title, headless.source->reader(), headless.arm(), headless.shown);
    velocity_kinematics_window body(panel_title, headless.source->reader(), headless.arm(), headless.shown, controls(), opening{jacobian_frame::body});

    CHECK_THAT(space.reading().message, ContainsSubstring("space Jacobian"));
    CHECK(space.reading().rows.empty());
    CHECK(body.reading().message.empty());
    CHECK(body.reading().rows.size() == matrix_rows + 2u * jacobian_block_count);
}

TEST_CASE("a published Jacobian reads as six unlabeled rows of one cell per joint in the matrix's own order", "[manipulator][window]")
{
    velocity_stage headless;
    headless.put(reading_of(Eigen::Vector3d(1.0, 0.5, 0.25)));

    velocity_kinematics_window panel(panel_title, headless.source->reader(), headless.arm(), headless.shown);
    const scene::readout shown = panel.reading();
    const jacobian taken       = six_by(2u, 1.0);

    REQUIRE(shown.rows.size() == matrix_rows + 2u * jacobian_block_count);
    for(Eigen::Index row = 0; row < 6; ++row)
    {
        const std::vector<scene::labeled_value> &cells = shown.rows[static_cast<std::size_t>(row)];
        REQUIRE(cells.size() == 2u);
        for(Eigen::Index column = 0; column < 2; ++column)
        {
            CHECK(cells[static_cast<std::size_t>(column)].label.empty());
            CHECK(cells[static_cast<std::size_t>(column)].value == Catch::Approx(static_cast<float>(taken(row, column))));
        }
    }
}

TEST_CASE("each block reads as its three singular values under its own name and its measure and condition number on the line beneath", "[manipulator][window]")
{
    velocity_stage headless;
    const Eigen::Vector3d values(2.0, 1.0, 0.5);
    headless.put(reading_of(values));

    velocity_kinematics_window panel(panel_title, headless.source->reader(), headless.arm(), headless.shown);
    const scene::readout shown = panel.reading();

    REQUIRE(shown.rows.size() == matrix_rows + 2u * jacobian_block_count);
    for(const std::size_t block : {0u, 1u})
    {
        const std::vector<scene::labeled_value> &carried = after_matrix(shown, 2u * block);
        REQUIRE(carried.size() == 3u);
        CHECK_THAT(carried.front().label, ContainsSubstring(block == 0u ? "Angular" : "Linear"));
        for(std::size_t axis = 0; axis < 3u; ++axis)
            CHECK(carried[axis].value == Catch::Approx(static_cast<float>(values[static_cast<Eigen::Index>(axis)])));

        const std::vector<scene::labeled_value> &beneath = after_matrix(shown, 2u * block + 1u);
        REQUIRE(beneath.size() == 2u);
        CHECK(beneath.front().value == Catch::Approx(static_cast<float>(values.prod())));
        CHECK(beneath.back().value == Catch::Approx(4.0f));
        CHECK(beneath.back().stated.empty());
    }
}

TEST_CASE("a values row names its block on its first cell alone, and the scalars row beneath it carries no block name", "[manipulator][window]")
{
    velocity_stage headless;
    headless.put(reading_of(Eigen::Vector3d(2.0, 1.0, 0.5)));

    velocity_kinematics_window panel(panel_title, headless.source->reader(), headless.arm(), headless.shown);
    const scene::readout shown = panel.reading();

    for(const std::size_t block : {0u, 1u})
    {
        const char *const named                          = block == 0u ? "Angular" : "Linear";
        const std::vector<scene::labeled_value> &carried = after_matrix(shown, 2u * block);
        REQUIRE(carried.size() == 3u);
        CHECK_FALSE(carried[0].label.empty());
        CHECK(carried[1].label.empty());
        CHECK(carried[2].label.empty());

        for(const scene::labeled_value &cell : after_matrix(shown, 2u * block + 1u))
        {
            CHECK_FALSE(cell.label.empty());
            CHECK(cell.label.find(named) == std::string::npos);
        }
    }
}

TEST_CASE("a condition number the publication does not carry is stated rather than printed, and no cell carries a value that is not finite", "[manipulator][window]")
{
    velocity_stage headless;
    headless.put(reading_of(Eigen::Vector3d(1.0, 0.5, 0.0)));

    velocity_kinematics_window panel(panel_title, headless.source->reader(), headless.arm(), headless.shown);
    const scene::readout shown = panel.reading();

    const scene::labeled_value &condition = after_matrix(shown, 1u).back();
    CHECK_FALSE(condition.stated.empty());
    CHECK(condition.value == 0.0f);
    for(const std::vector<scene::labeled_value> &row : shown.rows)
        for(const scene::labeled_value &cell : row)
            CHECK(std::isfinite(cell.value));
}

TEST_CASE("a block that is a refusal says so instead of standing a zero, and the other block still carries its numbers", "[manipulator][window]")
{
    velocity_stage headless;
    const Eigen::Vector3d values(2.0, 1.0, 0.5);
    const jacobian_manipulability halved{unexpected(refusal::unsupported_input), decomposed(values)};
    headless.put(reading_of(six_by(2u, 1.0), six_by(2u, 100.0), halved, halved));

    velocity_kinematics_window panel(panel_title, headless.source->reader(), headless.arm(), headless.shown);
    const scene::readout shown = panel.reading();

    REQUIRE(shown.rows.size() == matrix_rows + 3u);
    REQUIRE(after_matrix(shown, 0u).size() == 1u);
    CHECK_FALSE(after_matrix(shown, 0u).front().stated.empty());
    CHECK(after_matrix(shown, 1u).size() == 3u);
    CHECK(after_matrix(shown, 1u).front().value == Catch::Approx(2.0f));
    CHECK(after_matrix(shown, 2u).size() == 2u);
}

TEST_CASE("a decomposition whose two blocks are both refusals answers a message and no rows", "[manipulator][window]")
{
    velocity_stage headless;
    headless.put(reading_of(six_by(2u, 1.0), six_by(2u, 100.0), refused(), refused()));

    velocity_kinematics_window panel(panel_title, headless.source->reader(), headless.arm(), headless.shown);

    CHECK_THAT(panel.reading().message, ContainsSubstring("Neither block"));
    CHECK(panel.reading().rows.empty());
}

// Every drawn extent is the stencil's own, told there by whoever owns that control, so the window
// standing over it opens the drawings the composition named without moving any of them.
TEST_CASE("every value reaches the stencil at initialize even where the composition asked for no control at all, and no drawn extent moves", "[manipulator][window]")
{
    velocity_stage headless;
    opening named;
    named.frame             = jacobian_frame::body;
    named.reading           = ellipsoid_view::force;
    named.angular_ellipsoid = false;
    named.columns           = false;
    named.capped            = false;

    velocity_kinematics_window panel(panel_title, headless.source->reader(), headless.arm(), headless.shown, no_control(), named);
    panel.initialize();
    headless.draw();

    CHECK(headless.shown.jacobian_frame_shown() == jacobian_frame::body);
    CHECK(headless.shown.ellipsoid_scale(jacobian_block::angular) == Catch::Approx(angular_scale));
    CHECK(headless.shown.ellipsoid_scale(jacobian_block::linear) == Catch::Approx(linear_scale));
    CHECK(headless.shown.column_scale(jacobian_block::angular) == Catch::Approx(angular_column_scale));
    CHECK(headless.shown.column_scale(jacobian_block::linear) == Catch::Approx(linear_column_scale));
    CHECK(headless.shown.force_cap_ratio() == Catch::Approx(cap_ratio));
    CHECK_FALSE(headless.shown.force_capped());
    CHECK_FALSE(drawn(headless.body(jacobian_block::angular)));
    CHECK(drawn(headless.body(jacobian_block::linear)));
    CHECK_FALSE(drawn(headless.arrow(0u, jacobian_block::linear)));
}

// The reading asks the stencil for each block's extent where it needs one, so a scale moved after the
// window was built reaches the number a reader sees; a copy taken at construction could not, and the
// number read and the body drawn could then stand at different scales.
TEST_CASE("the reading follows a scale the stencil was told after the window was built, block by block", "[manipulator][window]")
{
    velocity_stage headless;
    headless.put(reading_of(Eigen::Vector3d(2.0, 1.0, 0.5)));

    velocity_kinematics_window panel(panel_title, headless.source->reader(), headless.arm(), headless.shown);

    REQUIRE(after_matrix(panel.reading(), 1u).size() == 2u);

    headless.shown.set_ellipsoid_scale(jacobian_block::angular, std::numeric_limits<double>::max());

    CHECK_THAT(after_matrix(panel.reading(), 1u).back().stated, ContainsSubstring("unbounded"));
    CHECK(after_matrix(panel.reading(), 3u).size() == 2u);
}

TEST_CASE("one control chooses the frame, and it governs the matrix read, the ellipsoids and the drawn columns together", "[manipulator][window]")
{
    velocity_stage headless;
    headless.put(reading_of(Eigen::Vector3d(1.0, 0.5, 0.25)));

    velocity_kinematics_window panel(panel_title, headless.source->reader(), headless.arm(), headless.shown, controls(), opening{});
    panel.initialize();
    headless.draw();

    const Eigen::Vector3d space_anchor = arrow_at(headless.arrow(0u, jacobian_block::linear));
    REQUIRE(panel.reading().rows.front().front().value == Catch::Approx(1.0f));

    choose_entry(panel, jacobian_row, 1);
    headless.draw();

    CHECK(headless.shown.jacobian_frame_shown() == jacobian_frame::body);
    CHECK(panel.reading().rows.front().front().value == Catch::Approx(100.0f));
    CHECK((arrow_at(headless.arrow(0u, jacobian_block::linear)) - space_anchor).norm() > 1.0e-6);
}

TEST_CASE("one control puts the force ellipsoid in place of the velocity one", "[manipulator][window]")
{
    velocity_stage headless;
    headless.put(reading_of(Eigen::Vector3d(2.0, 1.0, 0.5)));

    velocity_kinematics_window panel(panel_title, headless.source->reader(), headless.arm(), headless.shown, controls(), opening{});
    panel.initialize();
    headless.draw();
    const Eigen::Vector3d velocity_extent = body_extents(headless.body(jacobian_block::linear));

    choose_entry(panel, reading_row, 1);
    headless.draw();

    CHECK(panel.state().reading == ellipsoid_view::force);
    CHECK((body_extents(headless.body(jacobian_block::linear)) - velocity_extent).norm() > placed_back);
}

TEST_CASE("each of the four switches writes its own thing about what is drawn and reaches none of the others", "[manipulator][window]")
{
    velocity_stage headless;
    headless.put(reading_of(Eigen::Vector3d(1.0, 0.5, 0.25)));

    for(const int row : {angular_row, linear_row, columns_row, capped_row})
    {
        INFO("row " << row);
        velocity_kinematics_window panel(panel_title, headless.source->reader(), headless.arm(), headless.shown, controls(), opening{});
        panel.initialize();
        press(panel, row);
        headless.draw();

        CHECK(drawn(headless.body(jacobian_block::angular)) == (row != angular_row));
        CHECK(drawn(headless.body(jacobian_block::linear)) == (row != linear_row));
        CHECK(drawn(headless.arrow(0u, jacobian_block::angular)) == (row != columns_row));
        CHECK(headless.shown.force_capped() == (row != capped_row));
    }
}

// The two tones are neighbours by construction, so which part is which is not answerable from the
// picture. The key answers it, in the tones the arrows themselves wear rather than in a pair of
// literals that happen to agree with them.
TEST_CASE("a panel whose column switch is on names each part in the tone that part's arrows wear, and one whose switch is off names neither", "[manipulator][window]")
{
    velocity_stage headless;
    headless.put(reading_of(Eigen::Vector3d(1.0, 0.5, 0.25)));

    const ImU32 angular_tone = as_written(headless.arrow(0u, jacobian_block::angular));
    const ImU32 linear_tone  = as_written(headless.arrow(0u, jacobian_block::linear));
    REQUIRE(angular_tone != linear_tone);

    opening hidden;
    hidden.columns = false;
    velocity_kinematics_window standing(panel_title, headless.source->reader(), headless.arm(), headless.shown, controls(), opening{});
    velocity_kinematics_window without(panel_title, headless.source->reader(), headless.arm(), headless.shown, controls(), hidden);

    const std::vector<ImU32> drawn = tones_drawn(standing);
    const std::vector<ImU32> bare  = tones_drawn(without);

    CHECK(carries(drawn, angular_tone));
    CHECK(carries(drawn, linear_tone));
    CHECK_FALSE(carries(bare, angular_tone));
    CHECK_FALSE(carries(bare, linear_tone));
}

// The key stands with the switch it belongs to, so a composition that asked for no switch gets no key
// either, however the drawing it never offered a control for happens to open.
TEST_CASE("a panel whose composition offered no drawing switches names no part, though its columns are drawn", "[manipulator][window]")
{
    velocity_stage headless;
    headless.put(reading_of(Eigen::Vector3d(1.0, 0.5, 0.25)));

    velocity_kinematics_window panel(panel_title, headless.source->reader(), headless.arm(), headless.shown, no_control(), opening{});
    const std::vector<ImU32> seen = tones_drawn(panel);

    REQUIRE(panel.state().columns);
    CHECK_FALSE(carries(seen, as_written(headless.arrow(0u, jacobian_block::angular))));
    CHECK_FALSE(carries(seen, as_written(headless.arrow(0u, jacobian_block::linear))));
}

TEST_CASE("turning the drawn columns off and on again returns the panel to the drawing it had, so the key carries no state of its own", "[manipulator][window]")
{
    velocity_stage headless;
    headless.put(reading_of(Eigen::Vector3d(1.0, 0.5, 0.25)));

    velocity_kinematics_window panel(panel_title, headless.source->reader(), headless.arm(), headless.shown, controls(), opening{});
    panel.initialize();
    const std::size_t standing = signature_of(panel);

    press(panel, columns_row);
    const std::size_t without = signature_of(panel);

    press(panel, columns_row);

    CHECK(without != standing);
    CHECK(signature_of(panel) == standing);
}

TEST_CASE("a settings value round-trips through the window", "[manipulator][window]")
{
    velocity_stage headless;
    opening named;
    named.reading = ellipsoid_view::force;
    named.capped  = false;

    velocity_kinematics_window panel(panel_title, headless.source->reader(), headless.arm(), headless.shown, no_control(), named, std::string(velocity_at));
    const opening answered = panel.state();

    CHECK(answered.reading == ellipsoid_view::force);
    CHECK_FALSE(answered.capped);
    CHECK(answered.frame == jacobian_frame::space);
    CHECK(answered.columns);
    CHECK(panel.settings_path() == velocity_at);
    CHECK(panel.as_configurable() == &panel);
}

TEST_CASE("a window no key path was named for offers nothing to write", "[manipulator][window]")
{
    velocity_stage headless;
    velocity_kinematics_window panel(panel_title, headless.source->reader(), headless.arm(), headless.shown);

    CHECK(panel.settings_path().empty());
    CHECK(panel.as_configurable() == nullptr);
}

TEST_CASE("a force reading whose smallest singular value is zero says so beside that block and is named once however many readings stand", "[manipulator][window]")
{
    velocity_stage headless;
    headless.put(reading_of(Eigen::Vector3d(1.0, 0.5, 0.0)));

    opening named;
    named.reading = ellipsoid_view::force;
    velocity_kinematics_window panel(panel_title, headless.source->reader(), headless.arm(), headless.shown, no_control(), named);

    const std::string said = reported_by(
            [&]
            {
                for(std::size_t again = 0; again < 4u; ++again)
                    static_cast<void>(panel.reading());

                static_cast<void>(headless.loop.drain());
            });

    CHECK_THAT(after_matrix(panel.reading(), 1u).back().stated, ContainsSubstring("unbounded"));
    CHECK(said.find(velocity_kinematics_window::unbounded_ellipsoid) != std::string::npos);
    CHECK(said.find(velocity_kinematics_window::unbounded_ellipsoid) == said.rfind(velocity_kinematics_window::unbounded_ellipsoid));
}
