#include "opened_arm.h"
#include "described_arm.h"
#include "saved_document.h"
#include "composed_panels.h"
#include "scratch_directory.h"

#include "praxis/presets/arm.h"
#include "praxis/presets/arm_registration.h"

#include "praxis/manipulator/edited_pose.h"

#include "praxis/scene/preset.h"
#include "praxis/scene/imgui_window.h"

#include "praxis/config/store.h"
#include "praxis/config/writer.h"
#include "praxis/config/binding.h"
#include "praxis/config/document.h"
#include "praxis/config/configurable.h"

#include "praxis/rigid_motion/axis_order.h"

#include <catch2/catch_test_macros.hpp>

#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <algorithm>
#include <filesystem>

using namespace praxis;

namespace {

constexpr const char *numerical = "numerical inverse kinematics";

// A document naming its own starts leaves the target pose the only window with anything to offer.
constexpr const char *named_starts = "<ik_seeds><start index=\"1\" joints=\"0 0 0 0 0 0\"/></ik_seeds>";

// The Target pose window draws its control mode and its trajectory above the position, whose
// components stand one to a row.
constexpr std::size_t position_row = 2;

config::location target_document(const char *named)
{
    const std::filesystem::path directory = fixture::shared_scratch_directory() / named;
    std::filesystem::create_directories(directory);

    return fixture::arm_document(directory, "target.xml", fixture::arm_body(numerical, named_starts));
}

std::shared_ptr<scene::preset> opened(fixture::opened_arm &stage, const config::document &carried, const std::vector<std::filesystem::path> &roots)
{
    const presets::arm_scenario chosen      = presets::read_arm(carried, roots);
    std::shared_ptr<scene::preset> composed = stage.open(chosen, presets::arm_windows_numerical_ik(chosen));
    REQUIRE(composed != nullptr);
    REQUIRE(stage.loop.drain().has_value());

    return composed;
}

void type_position(const std::shared_ptr<scene::preset> &composed, std::size_t component, const char *typed)
{
    scene::imgui_window &panel = *fixture::panel_named(composed, "Target pose");
    static_cast<void>(fixture::geometry_of(panel));
    fixture::type_at(panel, position_row + component, typed);
}

std::vector<const config::configurable *> shown_by(const scene::preset &composed)
{
    std::vector<const config::configurable *> shown;
    for(const std::shared_ptr<scene::imgui_window> &panel : composed.windows)
        if(const config::configurable *one = panel->as_configurable(); one != nullptr)
            shown.push_back(one);

    return shown;
}

// A session typing the X component of the target pose and saving what it offers.
config::document saved_first_session(const config::location &at, const std::vector<std::filesystem::path> &roots)
{
    const config::document carried = fixture::read_arm_document(at);

    fixture::opened_arm stage;
    const std::shared_ptr<scene::preset> composed = opened(stage, carried, roots);
    type_position(composed, 0u, "0.125");

    return fixture::saved_into(at, fixture::offered_by(*composed, carried));
}

bool offers(const std::vector<config::edit> &offered, const std::string &key, const std::string &value)
{
    return std::ranges::count_if(offered, [&](const config::edit &one) { return one.key == key && one.value == value; }) == 1;
}

}

TEST_CASE("a target pose typed into the inverse kinematics scenario is saved beside the start and read back held", "[presets][documents]")
{
    const fixture::described_arm described(6, "six");
    const std::vector<std::filesystem::path> roots{described.directory};
    const config::location at      = target_document("typed_target_pose");
    const config::document carried = fixture::read_arm_document(at);

    fixture::opened_arm stage;
    const std::shared_ptr<scene::preset> composed = opened(stage, carried, roots);
    type_position(composed, 0u, "0.125");

    const std::vector<config::edit> offered = fixture::offered_by(*composed, carried);
    REQUIRE(offered.size() == 7u);
    CHECK(std::ranges::all_of(offered, [](const config::edit &one) { return one.key.starts_with("tool_pose/"); }));
    CHECK(offers(offered, "tool_pose/position/x", "0.125"));

    const config::document reread         = fixture::saved_into(at, offered);
    const presets::arm_scenario read_back = presets::read_arm(reread, roots);
    CHECK(read_back.tool_pose.standing == manipulator::pose_standing::held);
    CHECK(read_back.tool_pose.position.x() == 0.125f);
    CHECK(reread.origin_of("tool_pose/euler_order").kind == config::origin_kind::source);
    CHECK(read_back.tool_pose.order == axis_order::zyx);
}

TEST_CASE("an inverse kinematics scenario opened over a document naming no pose leaves nothing unsaved", "[presets][documents]")
{
    const fixture::described_arm described(6, "six");
    const config::location at      = target_document("seeded_target_pose");
    const config::document carried = fixture::read_arm_document(at);

    fixture::opened_arm stage;
    const std::shared_ptr<scene::preset> composed = opened(stage, carried, {described.directory});
    for(const std::shared_ptr<scene::imgui_window> &panel : composed->windows)
        static_cast<void>(fixture::geometry_of(*panel));

    CHECK(fixture::offered_by(*composed, carried).empty());
    CHECK_FALSE(config::anything_unsaved(shown_by(*composed), carried));
}

TEST_CASE("a saved target pose wins over the seed when the scenario is opened again", "[presets][documents]")
{
    const fixture::described_arm described(6, "six");
    const std::vector<std::filesystem::path> roots{described.directory};
    const config::location at      = target_document("reopened_target_pose");
    const config::document carried = saved_first_session(at, roots);

    fixture::opened_arm stage;
    const std::shared_ptr<scene::preset> composed = opened(stage, carried, roots);
    type_position(composed, 1u, "0.375");

    const std::vector<config::edit> offered = fixture::offered_by(*composed, carried);
    REQUIRE(offered.size() == 1u);
    CHECK(offers(offered, "tool_pose/position/y", "0.375"));

    const presets::arm_scenario read_back = presets::read_arm(fixture::saved_into(at, offered), roots);
    CHECK(read_back.tool_pose.position.x() == 0.125f);
    CHECK(read_back.tool_pose.position.y() == 0.375f);
}
