#include "opened_arm.h"
#include "described_arm.h"
#include "saved_document.h"
#include "composed_panels.h"
#include "scratch_directory.h"

#include "praxis/presets/arm.h"
#include "praxis/presets/arm_registration.h"

#include "praxis/manipulator/edited_pose.h"
#include "praxis/manipulator/control_mode.h"
#include "praxis/manipulator/screw_jog_window.h"

#include "praxis/scene/preset.h"
#include "praxis/scene/imgui_window.h"

#include "praxis/config/store.h"
#include "praxis/config/writer.h"
#include "praxis/config/binding.h"
#include "praxis/config/document.h"
#include "praxis/config/configurable.h"

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <algorithm>
#include <filesystem>

using namespace praxis;

namespace {

constexpr const char *every_window = "every window";

// The tool jog window draws its control mode above the start pose's position row, and the screw jog
// window its control mode, the two pose fields and the order selector above Reset start.
constexpr std::size_t start_position_row = 1;
constexpr std::size_t reset_start_row    = 4;
// The screw's point, direction and pitch follow Reset start, one row each.
constexpr std::size_t screw_pitch_row = reset_start_row + 3;

// A position saved as text and read back as floats, compared in metres.
constexpr double saved_position_tolerance = 1.0e-4;

config::location every_window_document(const char *named, const std::string &beside = std::string())
{
    const std::filesystem::path directory = fixture::shared_scratch_directory() / named;
    std::filesystem::create_directories(directory);

    return fixture::arm_document(directory, "motion.xml", fixture::arm_body(every_window, beside));
}

std::shared_ptr<scene::preset> opened(fixture::opened_arm &stage, const config::document &carried, const std::vector<std::filesystem::path> &roots)
{
    const presets::arm_scenario chosen      = presets::read_arm(carried, roots);
    std::shared_ptr<scene::preset> composed = stage.open(chosen, presets::arm_windows(chosen));
    REQUIRE(composed != nullptr);
    REQUIRE(stage.loop.drain().has_value());

    return composed;
}

scene::imgui_window &window_named(const std::shared_ptr<scene::preset> &composed, const char *named)
{
    const std::shared_ptr<scene::imgui_window> panel = fixture::panel_named(composed, named);
    REQUIRE(panel != nullptr);

    return *panel;
}

void draw_motion_windows(const std::shared_ptr<scene::preset> &composed)
{
    for(const char *named : {"Task space", "Tool frame jog", "Screw jog"})
        static_cast<void>(fixture::geometry_of(window_named(composed, named)));
}

// Seven bound edits under the pose path, no two of them keyed alike.
bool the_pose_once(const std::vector<config::edit> &offered)
{
    const auto under_pose = [](const config::edit &one) { return one.key.starts_with("tool_pose/") && one.kind == config::edit_kind::bound; };
    const auto keyed_once = [&offered](const config::edit &one) { return std::ranges::count(offered, one.key, &config::edit::key) == 1; };

    return offered.size() == 7u && std::ranges::all_of(offered, under_pose) && std::ranges::all_of(offered, keyed_once);
}

std::vector<const config::configurable *> shown_by(const scene::preset &composed)
{
    std::vector<const config::configurable *> shown;
    for(const std::shared_ptr<scene::imgui_window> &panel : composed.windows)
        if(const config::configurable *one = panel->as_configurable(); one != nullptr)
            shown.push_back(one);

    return shown;
}

constexpr const char *saved_screw = "<screw_jog mode=\"simulation\" pitch=\"0.25\" theta=\"30\"><q x=\"0.1\" y=\"0\" z=\"0\"/><w x=\"0\" y=\"1\" z=\"0\"/></screw_jog>";

bool holds_saved_screw(const manipulator::screw_jog_window::settings &held)
{
    return held.mode == manipulator::control_mode::simulation && held.pitch == 0.25f && held.theta_degrees == 30.f && held.q == Eigen::Vector3f{0.1f, 0.f, 0.f} &&
            held.w == Eigen::Vector3f::UnitY();
}

bool offers(const std::vector<config::edit> &offered, const std::string &key, const std::string &value)
{
    return std::ranges::count_if(offered, [&](const config::edit &one) { return one.key == key && one.value == value; }) == 1;
}

}

TEST_CASE("the three motion windows of the every window scenario save the pose they share once", "[presets][documents]")
{
    const fixture::described_arm described(6, "six");
    const std::vector<std::filesystem::path> roots{described.directory};
    const config::location at      = every_window_document("typed_shared_pose");
    const config::document carried = fixture::read_arm_document(at);

    fixture::opened_arm stage;
    const std::shared_ptr<scene::preset> composed = opened(stage, carried, roots);
    draw_motion_windows(composed);
    fixture::type_component_at(window_named(composed, "Tool frame jog"), start_position_row, 0u, "0.125");

    const std::vector<config::edit> offered = fixture::offered_by(*composed, carried);
    INFO("offered " << offered.size());
    CHECK(the_pose_once(offered));
    CHECK(offers(offered, "tool_pose/position/x", "0.125"));

    const presets::arm_scenario read_back = presets::read_arm(fixture::saved_into(at, offered), roots);
    CHECK(read_back.tool_pose.standing == manipulator::pose_standing::held);
    CHECK(read_back.tool_pose.position.x() == 0.125f);
}

TEST_CASE("the every window scenario opened over a document naming no pose leaves nothing unsaved", "[presets][documents]")
{
    const fixture::described_arm described(6, "six");
    const config::location at      = every_window_document("seeded_shared_pose");
    const config::document carried = fixture::read_arm_document(at);

    fixture::opened_arm stage;
    const std::shared_ptr<scene::preset> composed = opened(stage, carried, {described.directory});
    for(const std::shared_ptr<scene::imgui_window> &panel : composed->windows)
        static_cast<void>(fixture::geometry_of(*panel));

    CHECK(fixture::offered_by(*composed, carried).empty());
    CHECK_FALSE(config::anything_unsaved(shown_by(*composed), carried));
}

TEST_CASE("a start pose reset in the screw jog window counts as set and is saved", "[presets][documents]")
{
    const fixture::described_arm described(6, "six");
    const std::vector<std::filesystem::path> roots{described.directory};
    const config::location at      = every_window_document("reset_shared_pose");
    const config::document carried = fixture::read_arm_document(at);

    fixture::opened_arm stage;
    const std::shared_ptr<scene::preset> composed = opened(stage, carried, roots);
    draw_motion_windows(composed);
    fixture::press_at(window_named(composed, "Screw jog"), reset_start_row);

    const std::vector<config::edit> offered = fixture::offered_by(*composed, carried);
    INFO("offered " << offered.size());
    CHECK(the_pose_once(offered));

    // The arm opens with every joint at zero and no tool offset, so it stands at the chain's home.
    const presets::arm_scenario read_back = presets::read_arm(fixture::saved_into(at, offered), roots);
    const Eigen::Vector3d home            = fixture::derived_chain(described.where).home.topRightCorner<3, 1>();
    CHECK(read_back.tool_pose.standing == manipulator::pose_standing::held);
    CHECK((read_back.tool_pose.position.cast<double>() - home).norm() < saved_position_tolerance);
}

TEST_CASE("an arm document carrying a screw opens the every window scenario's screw jog at it", "[presets][documents]")
{
    const fixture::described_arm described(6, "six");
    const std::vector<std::filesystem::path> roots{described.directory};
    const config::document carried = fixture::read_arm_document(every_window_document("saved_screw", saved_screw));
    CHECK(holds_saved_screw(presets::read_arm(carried, roots).screw_jog));

    fixture::opened_arm stage;
    const std::shared_ptr<scene::preset> composed = opened(stage, carried, roots);
    const auto *screw_jog                         = dynamic_cast<const manipulator::screw_jog_window *>(&window_named(composed, "Screw jog"));
    REQUIRE(screw_jog != nullptr);
    CHECK(holds_saved_screw(screw_jog->state()));

    for(const std::shared_ptr<scene::imgui_window> &panel : composed->windows)
        static_cast<void>(fixture::geometry_of(*panel));
    CHECK(fixture::offered_by(*composed, carried).empty());
    CHECK_FALSE(config::anything_unsaved(shown_by(*composed), carried));
}

TEST_CASE("the every window scenario keeps a screw moved in its screw jog window with the document", "[presets][documents]")
{
    const fixture::described_arm described(6, "six");
    const std::vector<std::filesystem::path> roots{described.directory};
    const config::location at      = every_window_document("kept_screw");
    const config::document carried = fixture::read_arm_document(at);

    fixture::opened_arm stage;
    const std::shared_ptr<scene::preset> composed = opened(stage, carried, roots);
    draw_motion_windows(composed);
    fixture::type_component_at(window_named(composed, "Screw jog"), screw_pitch_row, 0u, "0.25");

    const std::vector<config::edit> offered = fixture::offered_by(*composed, carried);
    INFO("offered " << offered.size());
    CHECK(offers(offered, "screw_jog/pitch", "0.25"));
    CHECK(presets::read_arm(fixture::saved_into(at, offered), roots).screw_jog.pitch == 0.25f);
}
