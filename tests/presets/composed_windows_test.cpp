#include "described_arm.h"
#include "composed_panels.h"

#include "praxis/presets/arm.h"
#include "praxis/presets/screw.h"
#include "praxis/presets/euler_rungs.h"
#include "praxis/presets/rotation_axis.h"

#include "praxis/manipulator/capabilities.h"
#include "praxis/manipulator/loadable_robot_stencil.h"

#include "praxis/trajectory/capabilities.h"

#include "praxis/rigid_motion/capabilities.h"
#include "praxis/rigid_motion/frame_stencil.h"

#include "praxis/scene/preset.h"
#include "praxis/scene/preset_site.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/scenes/Scene.hpp>

#include <memory>
#include <string>
#include <ranges>
#include <vector>
#include <cstddef>
#include <optional>
#include <filesystem>

using namespace praxis;
using namespace praxis::fixture;

namespace {

// The stencil hangs everything it raises in the scene the composition was given, so what a composer
// told it stands there under the name the stencil gives it.
threepp::Object3D *named_in(threepp::Scene &target, const std::string &name)
{
    return target.getObjectByName<threepp::Object3D>(name);
}

std::size_t composed_objects(const std::shared_ptr<scene::preset> &composed)
{
    REQUIRE(composed != nullptr);

    return static_cast<rigid_motion::frame_stencil &>(*composed->stencil).count();
}

// The arm scenario against the reference implementation of every capability it composes from, which
// is the binding the shipped machines are composed under. What it opens and what it draws are the
// caller's.
std::shared_ptr<scene::preset> composed_arm(const scene::preset_site &site, const presets::arm_scenario &chosen, const manipulator::arm_composition &opened)
{
    return presets::arm_preset(site, manipulator::baseline(), trajectory::baseline(), rigid_motion::baseline(), chosen, opened);
}

}

TEST_CASE("each rung composes a selector, the arrangement panel and the two readouts beside them", "[presets][windows]")
{
    const std::vector<std::string> shown{"Frame selector", "Frame parameters", "Rotation", "Transformation"};

    threepp::Scene target;
    REQUIRE(composed_windows(presets::euler_rung_preset(unwired(target), rigid_motion::baseline(), presets::euler_rung::single_frame)) == shown);

    threepp::Scene beside;
    REQUIRE(composed_windows(presets::euler_rung_preset(unwired(beside), rigid_motion::baseline(), presets::euler_rung::paired_frames)) == shown);

    // The document is what an arrangement opens at, not what it offers: a key path with nothing
    // behind it shows the same windows as no arrangement at all.
    threepp::Scene without;
    REQUIRE(composed_windows(presets::euler_rung_preset(unwired(without), rigid_motion::baseline(), presets::euler_rung::single_frame,
                                                        presets::arrangement_source{"arrangement", std::nullopt})) == shown);
}

TEST_CASE("every window a rung composes opens exactly one panel under its own title", "[presets][windows]")
{
    threepp::Scene target;
    each_window_opens_one_panel(presets::euler_rung_preset(unwired(target), rigid_motion::baseline(), presets::euler_rung::single_frame));

    threepp::Scene beside;
    each_window_opens_one_panel(presets::euler_rung_preset(unwired(beside), rigid_motion::baseline(), presets::euler_rung::paired_frames));
}

TEST_CASE("the screw arrangement composes one window, and it is the screw", "[presets][windows]")
{
    const std::vector<std::string> shown{"Screw"};

    threepp::Scene target;
    const std::shared_ptr<scene::preset> composed = presets::screw_preset(unwired(target), rigid_motion::baseline());

    REQUIRE(composed_windows(composed) == shown);

    // The axis, the body driven about it, the threads that body travels and the located point the
    // axis passes through: a scenario that lost one of the four named objects reads here rather than
    // in the window list. The frame standing at the origin takes no index and is not one of them.
    REQUIRE(composed_objects(composed) == 4u);
}

TEST_CASE("every window the screw arrangement composes opens exactly one panel under its own title", "[presets][windows]")
{
    threepp::Scene target;
    each_window_opens_one_panel(presets::screw_preset(unwired(target), rigid_motion::baseline()));
}

TEST_CASE("the rotation arrangement composes one window, and it is the rotation", "[presets][windows]")
{
    const std::vector<std::string> shown{"Rotation"};

    threepp::Scene target;
    const std::shared_ptr<scene::preset> composed = presets::rotation_axis_preset(unwired(target), rigid_motion::baseline());

    REQUIRE(composed_windows(composed) == shown);

    // The frame the controls turn, the two arrows standing on the axis beside it, and one object per
    // arc the frame's own arrows trace. The frame stands at the origin, so the arrangement names no
    // fixed frame and there is no second triad standing in the same place.
    REQUIRE(composed_objects(composed) == 6u);
    REQUIRE(static_cast<rigid_motion::frame_stencil &>(*composed->stencil).fixed_frame_name().empty());
}

TEST_CASE("every window the rotation arrangement composes opens exactly one panel under its own title", "[presets][windows]")
{
    threepp::Scene target;
    each_window_opens_one_panel(presets::rotation_axis_preset(unwired(target), rigid_motion::baseline()));
}

TEST_CASE("the arm scenario composes ten windows, under the titles it gives them", "[presets][windows]")
{
    const std::vector<std::string> shown{"World object settings", "Joint control", "Task space", "Tool frame jog", "Screw jog",
                                         "Control parameters",    "Pose##1",       "Pose##2",    "Tool settings",  "Recording"};

    const described_arm described(1, "one_link");
    const presets::arm_scenario chosen = described_by(described.where);

    threepp::Scene target;
    REQUIRE(composed_windows(composed_arm(unwired(target), chosen, presets::arm_windows(chosen))) == shown);
}

TEST_CASE("every window the arm scenario composes opens exactly one panel under its own title", "[presets][windows]")
{
    const described_arm described(1, "one_link");
    const presets::arm_scenario chosen = described_by(described.where);

    threepp::Scene target;
    each_window_opens_one_panel(composed_arm(unwired(target), chosen, presets::arm_windows(chosen)));
}

TEST_CASE("the point-to-point scenario composes four windows, under the titles it gives them", "[presets][windows]")
{
    const std::vector<std::string> shown{"Joint control", "Waypoints", "Control parameters", "Preview"};

    const described_arm described(6, "six");
    const presets::arm_scenario chosen = described_by(described.where);

    threepp::Scene target;
    REQUIRE(composed_windows(composed_arm(unwired(target), chosen, presets::arm_windows_point_to_point(chosen))) == shown);
}

TEST_CASE("every window the point-to-point scenario composes opens exactly one panel under its own title", "[presets][windows]")
{
    const described_arm described(6, "six");
    const presets::arm_scenario chosen = described_by(described.where);

    threepp::Scene target;
    each_window_opens_one_panel(composed_arm(unwired(target), chosen, presets::arm_windows_point_to_point(chosen)));
}

TEST_CASE("the via-point scenario composes four windows, under the titles it gives them", "[presets][windows]")
{
    const std::vector<std::string> shown{"Joint control", "Waypoints", "Preview", "Joint curves"};

    const described_arm described(6, "six");
    const presets::arm_scenario chosen = described_by(described.where);

    threepp::Scene target;
    REQUIRE(composed_windows(composed_arm(unwired(target), chosen, presets::arm_windows_via_point(chosen))) == shown);
}

TEST_CASE("the path-comparison scenario composes three windows, under the titles it gives them", "[presets][windows]")
{
    const std::vector<std::string> shown{"Joint control", "Comparison", "View"};

    const described_arm described(6, "six");
    const presets::arm_scenario chosen = described_by(described.where);

    threepp::Scene target;
    REQUIRE(composed_windows(composed_arm(unwired(target), chosen, presets::arm_windows_path_comparison(chosen))) == shown);
}

// No list of waypoints and no plot of the path parameter, which is why three composers stand here
// rather than one that branches.
TEST_CASE("the path-comparison scenario opens neither the waypoint list nor the curve plot", "[presets][windows]")
{
    const described_arm described(6, "six");
    const presets::arm_scenario chosen = described_by(described.where);

    threepp::Scene target;
    const std::vector<std::string> shown = composed_windows(composed_arm(unwired(target), chosen, presets::arm_windows_path_comparison(chosen)));

    CHECK(std::ranges::find(shown, "Waypoints") == shown.end());
    CHECK(std::ranges::find(shown, "Joint curves") == shown.end());
}

TEST_CASE("every window the path-comparison scenario composes opens exactly one panel under its own title", "[presets][windows]")
{
    const described_arm described(6, "six");
    const presets::arm_scenario chosen = described_by(described.where);

    threepp::Scene target;
    each_window_opens_one_panel(composed_arm(unwired(target), chosen, presets::arm_windows_path_comparison(chosen)));
}

TEST_CASE("every window the via-point scenario composes opens exactly one panel under its own title", "[presets][windows]")
{
    const described_arm described(6, "six");
    const presets::arm_scenario chosen = described_by(described.where);

    threepp::Scene target;
    each_window_opens_one_panel(composed_arm(unwired(target), chosen, presets::arm_windows_via_point(chosen)));
}

TEST_CASE("the velocity-kinematics scenario composes four windows, under the titles it gives them", "[presets][windows]")
{
    const std::vector<std::string> shown{"Joint control", "Velocity kinematics", "Render controls", "View"};

    const described_arm described(6, "six");
    const presets::arm_scenario chosen = described_by(described.where);

    threepp::Scene target;
    REQUIRE(composed_windows(composed_arm(unwired(target), chosen, presets::arm_windows_velocity_kinematics(chosen))) == shown);
}

TEST_CASE("every window the velocity-kinematics scenario composes opens exactly one panel under its own title", "[presets][windows]")
{
    const described_arm described(6, "six");
    const presets::arm_scenario chosen = described_by(described.where);

    threepp::Scene target;
    each_window_opens_one_panel(composed_arm(unwired(target), chosen, presets::arm_windows_velocity_kinematics(chosen)));
}

// The composer tells the stencil how many columns to hold, from the joint count its own composition's
// derived chain gives, so a narrow arm stands the columns it has and no others.
TEST_CASE("the velocity-kinematics scenario composes over a two-joint arm, standing the columns that arm has", "[presets][windows]")
{
    const described_arm described(2, "two");
    const presets::arm_scenario chosen = described_by(described.where);

    threepp::Scene target;
    const std::shared_ptr<scene::preset> composed = composed_arm(unwired(target), chosen, presets::arm_windows_velocity_kinematics(chosen));
    REQUIRE(composed != nullptr);

    // The decoration reaches the scene where the running site would put it there, which is the only
    // point at which what the composer raised is reachable by name.
    REQUIRE(static_cast<manipulator::loadable_robot_stencil &>(*composed->stencil).initialize().has_value());

    for(const manipulator::jacobian_block part : {manipulator::jacobian_block::angular, manipulator::jacobian_block::linear})
    {
        CHECK(named_in(target, manipulator::loadable_robot_stencil::jacobian_column_name(0u, part)) != nullptr);
        CHECK(named_in(target, manipulator::loadable_robot_stencil::jacobian_column_name(1u, part)) != nullptr);
        CHECK(named_in(target, manipulator::loadable_robot_stencil::jacobian_column_name(2u, part)) == nullptr);
    }
}

TEST_CASE("no arm is composed from a description that does not load", "[presets][windows]")
{
    threepp::Scene target;
    const described_arm described(1, "one_link");
    const presets::arm_scenario chosen = described_by(described.directory / "no_such_description.urdf");

    REQUIRE(composed_arm(unwired(target), chosen, presets::arm_windows(chosen)) == nullptr);
}
