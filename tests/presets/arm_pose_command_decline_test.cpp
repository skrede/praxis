#include "captured_log.h"
#include "described_arm.h"
#include "composed_panels.h"

#include "praxis/presets/arm.h"

#include "praxis/manipulator/robot.h"
#include "praxis/manipulator/slots.h"
#include "praxis/manipulator/compose_arm.h"
#include "praxis/manipulator/capabilities.h"

#include "praxis/scene/preset.h"
#include "praxis/scene/preset_site.h"

#include "praxis/trajectory/capabilities.h"

#include "praxis/rigid_motion/capabilities.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/scenes/Scene.hpp>

#include <array>
#include <memory>
#include <string>
#include <cstddef>

using namespace praxis;
using namespace praxis::fixture;

namespace {

// A robot slot beside the name the descriptor table carries it under, which is the name a decline has
// to produce.
struct dependency
{
    manipulator::robot_slot slot;
    const char *named;
};

const std::array<dependency, 4> pose_transformations{dependency{manipulator::robot_slot::tool_pose_from_flange_pose, "robot.tool_pose_from_flange_pose"},
                                                     dependency{manipulator::robot_slot::flange_pose_from_tool_pose, "robot.flange_pose_from_tool_pose"},
                                                     dependency{manipulator::robot_slot::position_from_pose, "robot.position_from_pose"},
                                                     dependency{manipulator::robot_slot::orientation_from_pose, "robot.orientation_from_pose"}};

const std::array<dependency, 2> solves{dependency{manipulator::robot_slot::ik_solve_pose, "robot.ik_solve_pose"},
                                       dependency{manipulator::robot_slot::ik_solve_flange_pose, "robot.ik_solve_flange_pose"}};

// The compositions that open a window commanding a pose, each beside the name it reports under.
struct commanding_scenario
{
    manipulator::arm_composition (*opens)(presets::arm_scenario);
    const char *composer;
};

const std::array<commanding_scenario, 4> commanding{commanding_scenario{&presets::arm_windows, "presets.arm_windows"},
                                                    commanding_scenario{&presets::arm_windows_numerical_ik, "presets.arm_windows_numerical_ik"},
                                                    commanding_scenario{&presets::arm_windows_analytic_ik, "presets.arm_windows_analytic_ik"},
                                                    commanding_scenario{&presets::arm_windows_path_comparison, "presets.arm_windows_path_comparison"}};

manipulator::capabilities without(manipulator::capabilities composed, manipulator::robot_slot slot)
{
    if(slot == manipulator::robot_slot::tool_pose_from_flange_pose)
        composed.robot.tool_pose_from_flange_pose = &manipulator::inert::tool_pose_from_flange_pose;
    if(slot == manipulator::robot_slot::flange_pose_from_tool_pose)
        composed.robot.flange_pose_from_tool_pose = &manipulator::inert::flange_pose_from_tool_pose;
    if(slot == manipulator::robot_slot::position_from_pose)
        composed.robot.position_from_pose = &manipulator::inert::position_from_pose;
    if(slot == manipulator::robot_slot::orientation_from_pose)
        composed.robot.orientation_from_pose = &manipulator::inert::orientation_from_pose;
    if(slot == manipulator::robot_slot::ik_solve_pose)
        composed.robot.ik_solve_pose = &manipulator::inert::ik_solve_pose;
    if(slot == manipulator::robot_slot::ik_solve_flange_pose)
        composed.robot.ik_solve_flange_pose = &manipulator::inert::ik_solve_flange_pose;

    return composed;
}

// One composition beside what it said while it was made, which is the only place a decline is
// readable: it composes no window, so there is no panel to ask afterwards.
struct opening
{
    std::shared_ptr<scene::preset> composed;
    std::string reported;
};

opening open_with(threepp::Scene &target, const commanding_scenario &named, const presets::arm_scenario &chosen, const manipulator::capabilities &arm)
{
    opening made;
    {
        const tests::captured_log captured;
        made.composed = presets::arm_preset(unwired(target), arm, trajectory::baseline(), rigid_motion::baseline(), chosen, named.opens(chosen));
        made.reported = captured.text();
    }

    REQUIRE(made.composed != nullptr);

    return made;
}

void every_name_in_order(const std::string &reported)
{
    std::size_t stood = 0;
    for(const dependency &denied : pose_transformations)
    {
        INFO(denied.named);
        const std::size_t found = reported.find(denied.named);
        REQUIRE(found != std::string::npos);
        REQUIRE(found > stood);
        stood = found;
    }
}

}

// A slot that answers a plausible pose it never computed is caught once, at composition, and named.
// The three it was not denied are named nowhere, so a decline reporting the wrong one fails rather
// than reads.
TEST_CASE("a pose transformation left at its default composes no window that commands a pose, and is named", "[presets][windows]")
{
    const described_arm described(6, "pose_command_one_denied");
    const presets::arm_scenario chosen = described_by(described.where);

    for(const commanding_scenario &named : commanding)
        for(const dependency &denied : pose_transformations)
        {
            INFO(named.composer << " denied " << denied.named);

            threepp::Scene target;
            const opening made = open_with(target, named, chosen, without(manipulator::baseline(), denied.slot));

            REQUIRE(made.composed->windows.empty());
            REQUIRE(made.reported.find(denied.named) != std::string::npos);
            REQUIRE(made.reported.find(named.composer) != std::string::npos);

            for(const dependency &bound : pose_transformations)
                if(bound.slot != denied.slot)
                {
                    INFO(bound.named);
                    REQUIRE(made.reported.find(bound.named) == std::string::npos);
                }
        }
}

// The decline is read once, so it carries every name it owes rather than the first it found, and the
// order is the enumeration's rather than the order a search happened to take them in.
TEST_CASE("a decline names every pose transformation it was denied, in the order the enumeration declares them", "[presets][windows]")
{
    const described_arm described(6, "pose_command_none_bound");
    const presets::arm_scenario chosen = described_by(described.where);

    manipulator::capabilities none_of_them = manipulator::baseline();
    for(const dependency &denied : pose_transformations)
        none_of_them = without(none_of_them, denied.slot);

    for(const commanding_scenario &named : commanding)
    {
        INFO(named.composer);

        threepp::Scene target;
        const opening made = open_with(target, named, chosen, none_of_them);

        REQUIRE(made.composed->windows.empty());
        every_name_in_order(made.reported);
    }
}

TEST_CASE("every pose transformation bound composes the windows each scenario names and names no slot", "[presets][windows]")
{
    const described_arm described(6, "pose_command_all_bound");
    const presets::arm_scenario chosen = described_by(described.where);

    for(const commanding_scenario &named : commanding)
    {
        INFO(named.composer);

        threepp::Scene target;
        const opening made = open_with(target, named, chosen, manipulator::baseline());

        REQUIRE(!made.composed->windows.empty());
        for(const dependency &bound : pose_transformations)
        {
            INFO(bound.named);
            REQUIRE(made.reported.find(bound.named) == std::string::npos);
        }
    }
}

// A capability refuses where it is asked for, and none of these four asks for a solve while it is
// being composed: the solve is asked for through a window, after the composition stands.
TEST_CASE("an unbound solve declines no scenario that commands a pose", "[presets][windows]")
{
    const described_arm described(6, "pose_command_unsolved");
    const presets::arm_scenario chosen = described_by(described.where);

    for(const dependency &denied : solves)
        for(const commanding_scenario &named : commanding)
        {
            INFO(named.composer << " denied " << denied.named);

            threepp::Scene target;
            const opening made = open_with(target, named, chosen, without(manipulator::baseline(), denied.slot));

            REQUIRE(!made.composed->windows.empty());
            REQUIRE(made.reported.find(denied.named) == std::string::npos);
        }
}
