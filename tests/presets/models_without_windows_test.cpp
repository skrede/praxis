#include "opened_arm.h"
#include "drawn_lines.h"
#include "imgui_frame.h"
#include "panel_keys.h"
#include "carried_models.h"
#include "composed_panels.h"
#include "models_without_windows.h"

#include "praxis/presets/arm.h"

#include "praxis/manipulator/loadable_robot_stencil.h"

#include "praxis/rigid_motion/capabilities.h"

#include "praxis/scene/preset.h"
#include "praxis/scene/imgui_window.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/core/Object3D.hpp>

#include <Eigen/Core>

#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <utility>

using namespace praxis;
using namespace praxis::fixture;

namespace {

std::shared_ptr<scene::preset> opened_leaving_out(opened_arm &stage, const presets::arm_scenario &chosen, std::vector<std::string> dropped)
{
    return opened_under(stage, chosen, leaving_out(presets::arm_windows_tooling(chosen), std::move(dropped)), rigid_motion::baseline());
}

std::size_t offered_controls(scene::imgui_window &panel)
{
    tests::imgui_frame counting;

    return navigable_items(counting, [&panel] { panel.render(); });
}

double turned_apart(const threepp::Quaternion &first, const threepp::Quaternion &second)
{
    const Eigen::Vector4d apart(first.x - second.x, first.y - second.y, first.z - second.z, first.w - second.w);

    return apart.cwiseAbs().maxCoeff();
}

const Eigen::Vector3f kinematics_offset(0.1f, 0.2f, 0.3f);

}

TEST_CASE("the tooling composition without its tool window seats the tool where the one with it does", "[presets][windows]")
{
    const described_arm described(6, "six");
    const written_model tool("praxis_seated_tool.stl", 0.1f);
    const written_model world("praxis_seated_tool_world.stl", 0.2f);
    const presets::arm_scenario chosen = placing_models(described.where, tool.where, world.where, true);

    opened_arm windowed;
    opened_arm windowless;
    const std::shared_ptr<scene::preset> with_window    = opened_leaving_out(windowed, chosen, {});
    const std::shared_ptr<scene::preset> without_window = opened_leaving_out(windowless, chosen, {"Tool"});
    REQUIRE(panel_named(without_window, "Tool") == nullptr);

    threepp::Object3D *held = stencil_of(*without_window).attached_at(manipulator::flange_attachment::tool).get();
    threepp::Object3D *kept = stencil_of(*with_window).attached_at(manipulator::flange_attachment::tool).get();
    REQUIRE(held != nullptr);
    REQUIRE(kept != nullptr);
    CHECK(largest_difference(*held->matrixWorld, *kept->matrixWorld) < read_back);

    threepp::Object3D *flange = stencil_of(*without_window).attached_at(manipulator::flange_attachment::frame_marker).get();
    CHECK((placed_at(*held) - placed_at(*flange)).norm() > read_back);
    CHECK(std::abs(marker_separation(*without_window) - marker_separation(*with_window)) < read_back);
    CHECK(std::abs(marker_separation(*without_window) - static_cast<double>(kinematics_offset.norm())) < read_back);
}

TEST_CASE("the tooling composition without its world object window places the world object where the one with it does", "[presets][windows]")
{
    const described_arm described(6, "six");
    const written_model tool("praxis_placed_world_tool.stl", 0.1f);
    const written_model world("praxis_placed_world.stl", 0.2f);
    const presets::arm_scenario chosen = placing_models(described.where, tool.where, world.where, true);

    opened_arm windowed;
    opened_arm windowless;
    const std::shared_ptr<scene::preset> with_window    = opened_leaving_out(windowed, chosen, {});
    const std::shared_ptr<scene::preset> without_window = opened_leaving_out(windowless, chosen, {"World object"});
    REQUIRE(panel_named(without_window, "World object") == nullptr);

    const std::shared_ptr<threepp::Object3D> held = stencil_of(*without_window).world_object();
    const std::shared_ptr<threepp::Object3D> kept = stencil_of(*with_window).world_object();
    REQUIRE(held != nullptr);
    REQUIRE(kept != nullptr);
    CHECK(held->position.distanceTo(kept->position) < read_back);
    CHECK(held->scale.distanceTo(kept->scale) < read_back);
    CHECK(turned_apart(held->quaternion, kept->quaternion) < read_back);
    CHECK(held->position.distanceTo(threepp::Vector3(0.4f, -0.3f, 0.2f)) < read_back);
}

TEST_CASE("an inactive tool and world object composed without their windows stand nowhere", "[presets][windows]")
{
    const described_arm described(6, "six");
    const written_model tool("praxis_inactive_tool.stl", 0.1f);
    const written_model world("praxis_inactive_world.stl", 0.2f);
    const presets::arm_scenario chosen = placing_models(described.where, tool.where, world.where, false);

    opened_arm built;
    const std::shared_ptr<scene::preset> composed = opened_leaving_out(built, chosen, {"Tool", "World object"});

    CHECK(stencil_of(*composed).attached_at(manipulator::flange_attachment::tool) == nullptr);
    CHECK(stencil_of(*composed).world_object() == nullptr);
    CHECK(marker_separation(*composed) < read_back);
}

TEST_CASE("an inactive tool and world object composed with their windows are switched on through each window", "[presets][windows]")
{
    const described_arm described(6, "six");
    const written_model tool("praxis_switched_tool.stl", 0.1f);
    const written_model world("praxis_switched_world.stl", 0.2f);
    const presets::arm_scenario chosen = placing_models(described.where, tool.where, world.where, false);

    opened_arm built;
    const std::shared_ptr<scene::preset> composed = opened_leaving_out(built, chosen, {});
    REQUIRE(stencil_of(*composed).attached_at(manipulator::flange_attachment::tool) == nullptr);
    REQUIRE(stencil_of(*composed).world_object() == nullptr);

    press_at(*panel_named(composed, "Tool"), 0);
    press_at(*panel_named(composed, "World object"), 0);
    REQUIRE(built.loop.drain().has_value());
    built.draw(*composed);

    CHECK(stencil_of(*composed).attached_at(manipulator::flange_attachment::tool) != nullptr);
    CHECK(stencil_of(*composed).world_object() != nullptr);
    CHECK(std::abs(marker_separation(*composed) - static_cast<double>(kinematics_offset.norm())) < read_back);
}

TEST_CASE("the view window of the tooling composition without its tool window draws the stick to the seated tool frame", "[presets][windows]")
{
    const described_arm described(6, "six");
    const written_model tool("praxis_stick_without_window.stl", 0.1f);
    const written_model world("praxis_stick_without_window_world.stl", 0.2f);
    const presets::arm_scenario chosen = placing_models(described.where, tool.where, world.where, true);

    opened_arm windowed;
    opened_arm built;
    const std::shared_ptr<scene::preset> with_window = opened_leaving_out(windowed, chosen, {});
    const std::shared_ptr<scene::preset> composed    = opened_leaving_out(built, chosen, {"Tool"});
    CHECK(offered_controls(*panel_named(composed, "View")) == offered_controls(*panel_named(with_window, "View")));

    take_entry_at(*panel_named(composed, "View"), 4, 1);
    built.draw(*composed);

    manipulator::loadable_robot_stencil &stencil = stencil_of(*composed);
    const Eigen::Vector3d flange                 = placed_at(*stencil.attached_at(manipulator::flange_attachment::frame_marker));
    const Eigen::Vector3d frame                  = placed_at(*stencil.attached_at(manipulator::flange_attachment::tool_frame_marker));
    threepp::Object3D *stick                     = built.scene->getObjectByName<threepp::Object3D>(manipulator::loadable_robot_stencil::tool_stick_name());
    REQUIRE(drawn(stick));
    CHECK((placed_at(*stick) - 0.5 * (flange + frame)).norm() < read_back);
    CHECK(std::abs(static_cast<double>(stick->scale.y) - static_cast<double>(kinematics_offset.norm())) < read_back);
}
