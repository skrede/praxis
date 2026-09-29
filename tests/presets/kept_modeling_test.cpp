#include "opened_arm.h"
#include "described_arm.h"
#include "carried_models.h"
#include "labeled_panels.h"
#include "saved_document.h"
#include "composed_panels.h"
#include "scratch_directory.h"

#include "praxis/presets/screw_table.h"
#include "praxis/presets/kept_modeling.h"

#include "praxis/manipulator/tool_window.h"
#include "praxis/manipulator/robot_view_window.h"

#include "praxis/scene/preset.h"
#include "praxis/scene/imgui_window.h"

#include "praxis/config/error.h"
#include "praxis/config/store.h"
#include "praxis/config/binding.h"
#include "praxis/config/document.h"
#include "praxis/config/configurable.h"

#include "praxis/compat/expected.h"

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <utility>
#include <filesystem>

using namespace praxis;

namespace {

constexpr const char *described_frame = "robot_view/described_flange_frame";
constexpr const char *marker_scale    = "robot_view/frame_marker_scale";

config::binding keeping_at(const char *named, const std::string &beside)
{
    const std::filesystem::path directory = fixture::shared_scratch_directory() / named;
    std::filesystem::create_directories(directory);
    fixture::arm_document(directory, "chain.xml", "<screw_table><screws/>" + beside + "</screw_table>");

    return presets::screw_table_binding("chain.xml", directory);
}

// One kept opening handed to the arm preset the way a composer outside the library hands it.
struct kept_opening
{
    explicit kept_opening(presets::kept_modeling made)
            : opened(std::move(made))
            , composed(stage.open(opened.opening, opened.composed))
    {
        REQUIRE(composed != nullptr);
    }

    presets::kept_modeling opened;
    fixture::opened_arm stage;
    std::shared_ptr<scene::preset> composed;
};

std::shared_ptr<manipulator::robot_view_window> view_of(const std::shared_ptr<scene::preset> &composed)
{
    const auto held = std::dynamic_pointer_cast<manipulator::robot_view_window>(fixture::panel_named(composed, "View"));
    REQUIRE(held != nullptr);

    return held;
}

std::shared_ptr<manipulator::tool_window> tool_of(const std::shared_ptr<scene::preset> &composed)
{
    const auto held = std::dynamic_pointer_cast<manipulator::tool_window>(fixture::panel_named(composed, "Tool"));
    REQUIRE(held != nullptr);

    return held;
}

// The loader is the pane a window holding no tool opens at: the file field, then the button. A
// loaded tool puts the activation switch and the view choice above the offset's three rows.
void loaded_and_offset(const std::shared_ptr<scene::preset> &composed, const std::filesystem::path &model)
{
    scene::imgui_window &panel = *tool_of(composed);
    fixture::type_at(panel, 1u, model.string().c_str());
    fixture::press_at(panel, 2u);
    fixture::type_at(panel, 4u, "0.45");
}

void holds_tool(const manipulator::tool_window &panel, const std::filesystem::path &model)
{
    const manipulator::tool_window::settings held = panel.state();
    REQUIRE(held.active);
    REQUIRE(held.model_path == model.string());
    REQUIRE((held.kinematics_offset - Eigen::Vector3f(0.f, 0.f, 0.45f)).norm() < 1.0e-6f);
}

bool anything_unsaved(const kept_opening &kept)
{
    std::vector<const config::configurable *> shown;
    for(const std::shared_ptr<scene::imgui_window> &panel : kept.composed->windows)
        shown.push_back(panel->as_configurable());

    return config::anything_unsaved(shown, kept.opened.carried);
}

config::document reread(const config::binding &bound)
{
    const config::outcome read = config::load_or_defaults(bound);
    INFO((read.failure.has_value() ? read.failure->message : std::string()));
    REQUIRE_FALSE(read.failure.has_value());

    return read.values;
}

bool written_flag(const config::document &values, const char *key)
{
    INFO(key);
    const expected<bool, config::error> read = values.flag(key);
    REQUIRE(read.has_value());
    CHECK(values.origin_of(key).kind == config::origin_kind::source);

    return *read;
}

void saved_view_and_tool(const presets::arm_scenario &chosen, const config::binding &keeping, const std::filesystem::path &model)
{
    const kept_opening first(presets::arm_windows_kept_modeling(chosen, keeping, presets::modeling_beside::pose_and_tool));
    fixture::press_on(*view_of(first.composed), "Description's flange frame");
    loaded_and_offset(first.composed, model);
    const expected<void, config::error> written = config::save(first.opened.bound, fixture::offered_by(*first.composed, first.opened.carried));
    INFO((written.has_value() ? std::string() : written.error().message));
    REQUIRE(written.has_value());
}

}

TEST_CASE("a view switch and a tool saved into a kept chain document are what the windows reopen at", "[presets]")
{
    const fixture::described_arm described(6, "six");
    const fixture::written_model model("praxis_kept_modeling_tool.stl", 0.1f);
    const config::binding keeping      = keeping_at("kept_modeling_reopened", "");
    const presets::arm_scenario chosen = fixture::described_by(described.where);
    saved_view_and_tool(chosen, keeping, model.where);

    const kept_opening second(presets::arm_windows_kept_modeling(chosen, keeping, presets::modeling_beside::pose_and_tool));
    CHECK(second.opened.bound.at.resolved == keeping.at.resolved);
    CHECK(second.opened.opening.robot_view.described_frame_marker);
    CHECK(view_of(second.composed)->state().described_frame_marker);
    holds_tool(*tool_of(second.composed), model.where);
    CHECK(written_flag(reread(second.opened.bound), described_frame));
    CHECK_FALSE(anything_unsaved(second));
}

TEST_CASE("each view or tool value a kept chain document does not carry opens at the scenario's own", "[presets]")
{
    const fixture::described_arm described(6, "six");
    const Eigen::Vector3f offset(0.f, 0.f, 0.2f);
    presets::arm_scenario chosen             = fixture::described_by(described.where);
    chosen.robot_view.described_frame_marker = true;
    chosen.robot_view.marker_scale           = 2.0;
    chosen.tool.kinematics_offset            = offset;
    const config::binding keeping            = keeping_at("kept_modeling_fallbacks", "<robot_view frame_marker_scale=\"3\"/>");

    const kept_opening kept(presets::arm_windows_kept_modeling(chosen, keeping, presets::modeling_beside::pose_and_tool));
    const manipulator::robot_view_window::settings view = view_of(kept.composed)->state();
    CHECK(view.described_frame_marker);
    CHECK(std::abs(view.marker_scale - 3.0) < 1.0e-6);
    CHECK((tool_of(kept.composed)->state().kinematics_offset - offset).norm() < 1.0e-6f);
    CHECK(kept.opened.carried.origin_of(described_frame).kind == config::origin_kind::fallback);
    CHECK(kept.opened.carried.origin_of(marker_scale).kind == config::origin_kind::source);
}

TEST_CASE("a scenario naming view values of its own leaves nothing unsaved over an empty kept chain document", "[presets]")
{
    const fixture::described_arm described(6, "six");
    presets::arm_scenario chosen   = fixture::described_by(described.where);
    chosen.robot_view.axis_reach   = 0.3;
    chosen.robot_view.marker_scale = 2.0;
    const config::binding keeping  = keeping_at("kept_modeling_nothing_unsaved", "");

    const kept_opening kept(presets::arm_windows_kept_modeling(chosen, keeping));
    const manipulator::robot_view_window::settings view = view_of(kept.composed)->state();
    CHECK(std::abs(view.axis_reach.value_or(0.0) - 0.3) < 1.0e-6);
    CHECK(std::abs(view.marker_scale - 2.0) < 1.0e-6);
    CHECK(fixture::offered_by(*kept.composed, kept.opened.carried).empty());
    CHECK_FALSE(anything_unsaved(kept));
}
