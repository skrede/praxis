#include "opened_arm.h"
#include "drawn_lines.h"
#include "described_arm.h"
#include "carried_models.h"
#include "labeled_panels.h"
#include "saved_document.h"
#include "composed_panels.h"
#include "scratch_directory.h"

#include "praxis/presets/arm_registration.h"

#include "praxis/manipulator/tool_window.h"
#include "praxis/manipulator/robot_view_window.h"
#include "praxis/manipulator/loadable_robot_stencil.h"

#include "praxis/scene/preset.h"
#include "praxis/scene/imgui_window.h"
#include "praxis/scene/preset_registry.h"

#include "praxis/config/error.h"
#include "praxis/config/store.h"
#include "praxis/config/writer.h"
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
#include <filesystem>

using namespace praxis;

namespace {

constexpr const char *supplied_chain  = "supplied chain";
constexpr const char *chain_and_tool  = "supplied chain and tool";
constexpr const char *keeping_chain   = "<screw_table document=\"chain.xml\"/>";
constexpr const char *preview_view    = "<robot_view model=\"Meshes\" screw_axes=\"true\" axis_reach=\"0\"/>";
constexpr const char *described_frame = "robot_view/described_flange_frame";
constexpr const char *axes_off_view   = "<robot_view model=\"Meshes\" screw_axes=\"false\" axis_reach=\"0\"/>";
constexpr const char *screw_axes_key  = "robot_view/screw_axes";

std::filesystem::path scratch(const char *named)
{
    const std::filesystem::path directory = fixture::shared_scratch_directory() / named;
    std::filesystem::create_directories(directory);

    return directory;
}

config::location arm_at(const std::filesystem::path &directory, const std::string &scenario, const std::string &beside)
{
    return fixture::arm_document(directory, "arm.xml", fixture::arm_body(scenario, beside));
}

void chain_at(const std::filesystem::path &directory, const std::string &beside)
{
    fixture::arm_document(directory, "chain.xml", "<screw_table><screws/>" + beside + "</screw_table>");
}

// One arm document registered, each composition of it built through the registry on a strand of its
// own stage, and every binding and document those compositions announced, in order.
struct registered_arm
{
    registered_arm(const config::location &document, const fixture::described_arm &described)
            : registry(std::make_shared<scene::preset_registry>())
    {
        const presets::composed_route recording = [this](const config::binding &at, const config::document &values)
        {
            bindings.push_back(at);
            documents.push_back(values);
        };
        const std::vector<std::string> named =
                presets::register_arms(registry, std::vector<config::location>{document}, std::vector<std::filesystem::path>{described.directory}, {}, recording);
        REQUIRE(named.size() == 1u);
        name = named.front();
    }

    registered_arm(const registered_arm &) = delete;

    std::shared_ptr<scene::preset> composed(fixture::opened_arm &stage) const
    {
        const scene::preset_registry::factory compose = registry->load_preset(name);
        REQUIRE(compose != nullptr);

        std::shared_ptr<scene::preset> built = compose(stage.site());
        REQUIRE(built != nullptr);
        REQUIRE(built->initialize().has_value());
        REQUIRE(stage.loop.drain().has_value());

        return built;
    }

    std::shared_ptr<scene::preset_registry> registry;
    std::string name;
    std::vector<config::binding> bindings;
    std::vector<config::document> documents;
};

// Every window's offer written into the document the last composition announced, as a save writes it.
void saved(const registered_arm &arm, const scene::preset &composed)
{
    REQUIRE_FALSE(arm.bindings.empty());

    const expected<void, config::error> written = config::save(arm.bindings.back(), fixture::offered_by(composed, arm.documents.back()));
    INFO((written.has_value() ? std::string() : written.error().message));
    REQUIRE(written.has_value());
}

config::document reread(const registered_arm &arm)
{
    const config::outcome read = config::load_or_defaults(arm.bindings.back());
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

std::string written_text(const config::document &values, const char *key)
{
    INFO(key);
    const expected<std::string, config::error> read = values.text(key);
    REQUIRE(read.has_value());
    CHECK(values.origin_of(key).kind == config::origin_kind::source);

    return *read;
}

double written_real(const config::document &values, const char *key)
{
    INFO(key);
    const expected<double, config::error> read = values.real(key);
    REQUIRE(read.has_value());
    CHECK(values.origin_of(key).kind == config::origin_kind::source);

    return *read;
}

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

void holds_tool(const manipulator::tool_window &panel, const std::filesystem::path &model)
{
    const manipulator::tool_window::settings held = panel.state();
    REQUIRE(held.active);
    REQUIRE(held.model_path == model.string());
    REQUIRE((held.kinematics_offset - Eigen::Vector3f(0.f, 0.f, 0.45f)).norm() < 1.0e-6f);
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

// A held tool puts the activation switch above the view choice, the file field and the button.
void load_beside_held(scene::imgui_window &panel, const std::string &typed)
{
    fixture::stands_on(panel, "STL file");
    fixture::type_at(panel, 2u, typed.c_str());
    fixture::press_at(panel, 3u);
}

bool carries_tool(const scene::preset &composed)
{
    const auto drawn = std::dynamic_pointer_cast<manipulator::loadable_robot_stencil>(composed.stencil);
    REQUIRE(drawn != nullptr);

    return drawn->attached_at(manipulator::flange_attachment::tool) != nullptr;
}

bool axes_drawn(fixture::opened_arm &stage, const scene::preset &composed)
{
    stage.draw(composed);
    threepp::Object3D *axis = fixture::first_line_under(*stage.scene, manipulator::loadable_robot_stencil::joint_axis_name(0));
    REQUIRE(axis != nullptr);

    return fixture::drawn(axis);
}

bool anything_unsaved(const registered_arm &arm, const scene::preset &composed)
{
    std::vector<const config::configurable *> shown;
    for(const std::shared_ptr<scene::imgui_window> &panel : composed.windows)
        shown.push_back(panel->as_configurable());

    return config::anything_unsaved(shown, arm.documents.back());
}

}

TEST_CASE("the description flange frame switch turned on beside a kept chain is on again when the scenario is opened again", "[presets][registry]")
{
    const fixture::described_arm described(6, "six");
    const std::filesystem::path directory = scratch("kept_switch");
    chain_at(directory, "");
    registered_arm arm(arm_at(directory, supplied_chain, std::string(keeping_chain) + preview_view), described);

    fixture::opened_arm first;
    const std::shared_ptr<scene::preset> composed = arm.composed(first);
    fixture::press_on(*view_of(composed), "Description's flange frame");
    fixture::type_at(*fixture::panel_named(composed, "Chain"), 0u, "0.5");
    saved(arm, *composed);

    const config::document kept = reread(arm);
    CHECK(written_flag(kept, described_frame));
    CHECK(std::abs(written_real(kept, "screws/home/position/x") - 0.5) < 1.0e-9);

    fixture::opened_arm second;
    CHECK(view_of(arm.composed(second))->state().described_frame_marker);
}

TEST_CASE("a tool loaded and offset beside a kept chain is the tool the scenario opens at again", "[presets][registry]")
{
    const fixture::described_arm described(6, "six");
    const fixture::written_model model("praxis_kept_beside_tool.stl", 0.1f);
    const std::filesystem::path directory = scratch("kept_tool");
    chain_at(directory, "");
    registered_arm arm(arm_at(directory, chain_and_tool, std::string(keeping_chain) + "<tool active=\"false\" model=\"\"/>"), described);

    fixture::opened_arm first;
    const std::shared_ptr<scene::preset> composed = arm.composed(first);
    loaded_and_offset(composed, model.where);
    holds_tool(*tool_of(composed), model.where);
    saved(arm, *composed);

    fixture::opened_arm second;
    const std::shared_ptr<scene::preset> again = arm.composed(second);
    holds_tool(*tool_of(again), model.where);
    CHECK(carries_tool(*again));
}

TEST_CASE("a tool Load that fails beside a supplied chain saves no tool active and the path that failed, and the scenario opens again holding no tool", "[presets][registry]")
{
    const fixture::described_arm described(6, "six");
    const fixture::written_model model("praxis_failed_load_tool.stl", 0.1f);
    const std::filesystem::path directory = scratch("failed_load");
    const std::string absent              = (directory / "absent.stl").string();
    registered_arm arm(arm_at(directory, chain_and_tool, "<tool active=\"true\" model=\"" + model.where.string() + "\"/>"), described);

    fixture::opened_arm first;
    const std::shared_ptr<scene::preset> composed = arm.composed(first);
    REQUIRE(carries_tool(*composed));
    REQUIRE(tool_of(composed)->state().active);
    load_beside_held(*tool_of(composed), absent);
    CHECK_FALSE(carries_tool(*composed));
    saved(arm, *composed);
    CHECK_FALSE(written_flag(reread(arm), "tool/active"));
    CHECK(written_text(reread(arm), "tool/model") == absent);

    fixture::opened_arm second;
    const std::shared_ptr<scene::preset> again = arm.composed(second);
    CHECK_FALSE(carries_tool(*again));
    CHECK_FALSE(tool_of(again)->state().active);
    CHECK(tool_of(again)->state().model_path == absent);
    CHECK_FALSE(anything_unsaved(arm, *again));
}

TEST_CASE("the description flange frame switch beside a chain kept nowhere is saved into the arm document and opens again", "[presets][registry]")
{
    const fixture::described_arm described(6, "six");
    const std::filesystem::path directory = scratch("kept_nowhere_switch");
    const config::location document       = arm_at(directory, supplied_chain, preview_view);
    registered_arm arm(document, described);

    fixture::opened_arm first;
    const std::shared_ptr<scene::preset> composed = arm.composed(first);
    REQUIRE(arm.bindings.back().at.resolved == document.resolved);
    fixture::press_on(*view_of(composed), "Description's flange frame");
    saved(arm, *composed);
    CHECK(written_flag(reread(arm), described_frame));

    fixture::opened_arm second;
    CHECK(view_of(arm.composed(second))->state().described_frame_marker);
}

TEST_CASE("each view value a kept chain document does not carry opens at what the arm document names", "[presets][registry]")
{
    const fixture::described_arm described(6, "six");
    const std::filesystem::path directory = scratch("kept_view_values");
    chain_at(directory, "<robot_view frame_marker_scale=\"3\"/>");
    const std::string view = "<robot_view model=\"Meshes\" screw_axes=\"true\" axis_reach=\"0\" described_flange_frame=\"true\" frame_marker_scale=\"2\"/>";
    registered_arm arm(arm_at(directory, supplied_chain, keeping_chain + view), described);

    fixture::opened_arm stage;
    const manipulator::robot_view_window::settings opened = view_of(arm.composed(stage))->state();
    CHECK(opened.described_frame_marker);
    CHECK(std::abs(opened.marker_scale - 3.0) < 1.0e-6);
}

TEST_CASE("a switch neither document names opens off beside a kept chain", "[presets][registry]")
{
    const fixture::described_arm described(6, "six");
    const std::filesystem::path directory = scratch("kept_switch_unnamed");
    chain_at(directory, "");
    registered_arm arm(arm_at(directory, supplied_chain, std::string(keeping_chain) + preview_view), described);

    fixture::opened_arm stage;
    CHECK_FALSE(view_of(arm.composed(stage))->state().described_frame_marker);
}

TEST_CASE("an arm document naming view values of its own leaves nothing unsaved beside a kept chain at open", "[presets][registry]")
{
    const fixture::described_arm described(6, "six");
    const std::filesystem::path directory = scratch("kept_nothing_unsaved");
    chain_at(directory, "");
    const std::string view = "<robot_view model=\"Meshes\" screw_axes=\"true\" axis_reach=\"0.3\" frame_marker_scale=\"2\"/>";
    registered_arm arm(arm_at(directory, supplied_chain, keeping_chain + view), described);

    fixture::opened_arm stage;
    const std::shared_ptr<scene::preset> composed         = arm.composed(stage);
    const manipulator::robot_view_window::settings opened = view_of(composed)->state();
    CHECK(std::abs(opened.axis_reach.value_or(0.0) - 0.3) < 1.0e-6);
    CHECK(std::abs(opened.marker_scale - 2.0) < 1.0e-6);
    CHECK_FALSE(anything_unsaved(arm, *composed));
    CHECK(fixture::offered_by(*composed, arm.documents.back()).empty());
}

TEST_CASE("screw axes a supplied-chain document turns off stay off across a save, a reopen and a second save", "[presets][registry]")
{
    const fixture::described_arm described(6, "six");
    registered_arm arm(arm_at(scratch("kept_axes_off"), supplied_chain, axes_off_view), described);

    fixture::opened_arm first;
    const std::shared_ptr<scene::preset> composed = arm.composed(first);
    CHECK_FALSE(view_of(composed)->state().decoration);
    CHECK_FALSE(axes_drawn(first, *composed));
    fixture::press_on(*view_of(composed), "Description's flange frame");
    saved(arm, *composed);
    CHECK_FALSE(written_flag(reread(arm), screw_axes_key));

    fixture::opened_arm second;
    const std::shared_ptr<scene::preset> again = arm.composed(second);
    CHECK_FALSE(view_of(again)->state().decoration);
    CHECK_FALSE(axes_drawn(second, *again));
    fixture::press_on(*view_of(again), "Description's flange frame");
    saved(arm, *again);
    CHECK_FALSE(written_flag(reread(arm), screw_axes_key));

    fixture::opened_arm third;
    CHECK_FALSE(view_of(arm.composed(third))->state().decoration);
}

TEST_CASE("an arm document naming no screw axes opens the supplied-chain scenario with them drawn", "[presets][registry]")
{
    const fixture::described_arm described(6, "six");
    registered_arm arm(arm_at(scratch("axes_unnamed"), supplied_chain, ""), described);

    fixture::opened_arm stage;
    const std::shared_ptr<scene::preset> composed = arm.composed(stage);
    CHECK(view_of(composed)->state().decoration);
    CHECK(axes_drawn(stage, *composed));
}

TEST_CASE("the screw axes switch beside a supplied chain hides the axes and leaves the change unsaved until it is saved", "[presets][registry]")
{
    const fixture::described_arm described(6, "six");
    registered_arm arm(arm_at(scratch("axes_switched"), supplied_chain, preview_view), described);

    fixture::opened_arm stage;
    const std::shared_ptr<scene::preset> composed = arm.composed(stage);
    CHECK_FALSE(anything_unsaved(arm, *composed));
    fixture::press_on(*view_of(composed), "Screw axes");
    CHECK_FALSE(view_of(composed)->state().decoration);
    CHECK_FALSE(axes_drawn(stage, *composed));
    CHECK(anything_unsaved(arm, *composed));
    saved(arm, *composed);
    CHECK_FALSE(written_flag(reread(arm), screw_axes_key));

    fixture::opened_arm again;
    CHECK_FALSE(view_of(arm.composed(again))->state().decoration);
}
