#include "opened_arm.h"
#include "captured_log.h"
#include "carried_models.h"
#include "composed_panels.h"

#include "praxis/presets/arm.h"

#include "praxis/manipulator/robot.h"
#include "praxis/manipulator/tool_window.h"
#include "praxis/manipulator/capabilities.h"
#include "praxis/manipulator/baseline/robot.h"
#include "praxis/manipulator/world_object_window.h"
#include "praxis/manipulator/loadable_robot_stencil.h"

#include "praxis/scene/preset.h"
#include "praxis/scene/imgui_window.h"

#include "praxis/rigid_motion/axis_order.h"

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

#include <array>
#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <utility>
#include <optional>
#include <filesystem>
#include <system_error>

using namespace praxis;
using namespace praxis::fixture;

namespace {

const std::vector<std::string> &tooling_windows()
{
    static const std::vector<std::string> shown{"Joint control", "Pose", "Tool", "World object", "View"};

    return shown;
}

// Every pose the composed arm assembled, recorded where the arm itself reads one, so a case compares
// two readings the composition produced rather than a reading against a pose it computed for itself.
std::vector<transform> read_through;

Eigen::Vector3d recorded_position(const transform &pose)
{
    read_through.push_back(pose);

    return manipulator::position_from_pose(pose);
}

manipulator::capabilities recording_poses()
{
    manipulator::capabilities composed = manipulator::baseline();
    composed.robot.position_from_pose  = &recorded_position;

    return composed;
}

// The tool's pose is assembled before the flange's, which the snapshot's own initializer fixes.
std::pair<transform, transform> last_publication()
{
    REQUIRE(read_through.size() >= 2u);

    return {read_through[read_through.size() - 2u], read_through.back()};
}

// The four the scenario reads every pose through, each beside the name the descriptor table carries
// it under, which is the name a decline has to produce.
struct dependency
{
    manipulator::robot_slot slot;
    const char *named;
};

const std::array<dependency, 4> pose_transformations{dependency{manipulator::robot_slot::tool_pose_from_flange_pose, "robot.tool_pose_from_flange_pose"},
                                                     dependency{manipulator::robot_slot::flange_pose_from_tool_pose, "robot.flange_pose_from_tool_pose"},
                                                     dependency{manipulator::robot_slot::position_from_pose, "robot.position_from_pose"},
                                                     dependency{manipulator::robot_slot::orientation_from_pose, "robot.orientation_from_pose"}};

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

    return composed;
}

// The stage's own opener composes from the reference bindings alone, and every case below turns on
// what one composition was handed instead of them.
std::shared_ptr<scene::preset> open_with(opened_arm &stage, const presets::arm_scenario &chosen, const manipulator::capabilities &arm)
{
    std::shared_ptr<scene::preset> composed = presets::arm_preset(stage.site(), arm, trajectory::baseline(), rigid_motion::baseline(), chosen, presets::arm_windows_tooling(chosen));
    if(composed != nullptr)
        REQUIRE(composed->initialize().has_value());

    return composed;
}

// The offset is what puts the reported tool pose off the flange, which the cases below read; the
// rest of the scenario is the shared one.
presets::arm_scenario carrying(const std::filesystem::path &description, const std::filesystem::path &tool, const std::filesystem::path &world)
{
    presets::arm_scenario chosen  = carrying_models(description, tool, world);
    chosen.tool.kinematics_offset = Eigen::Vector3f(0.1f, 0.2f, 0.3f);

    return chosen;
}

}

// A fixture owns the directory it writes into rather than a name under the shared temporary root,
// so the name a caller asks for says nothing about where the document lands.
TEST_CASE("two descriptions asked for under one name stand apart", "[presets][fixtures]")
{
    const described_arm first(2, "same");
    REQUIRE(std::filesystem::exists(first.where));

    {
        const described_arm second(3, "same");
        REQUIRE(second.where != first.where);
        REQUIRE(std::filesystem::exists(second.where));
        REQUIRE(derived_chain(second.where).space_screws.size() == 3u);
    }

    REQUIRE(std::filesystem::exists(first.where));
    REQUIRE(derived_chain(first.where).space_screws.size() == 2u);
}

// The two fixtures a tooling case builds together hold separate directories, so removing either
// tree leaves the other's file where it was.
TEST_CASE("a written model outlives the description built beside it", "[presets][fixtures]")
{
    const written_model tool("praxis_tooling_apart.stl", 0.1f);
    REQUIRE(std::filesystem::exists(tool.where));

    {
        const described_arm described(2, "apart");
        REQUIRE(described.directory != tool.directory);
    }

    REQUIRE(std::filesystem::exists(tool.where));
}

// The removal is asked for through the error code rather than by throwing, so a directory taken
// away underneath a fixture leaves its destructor nothing to report.
TEST_CASE("a fixture whose directory is already gone destroys quietly", "[presets][fixtures]")
{
    std::filesystem::path taken;

    REQUIRE_NOTHROW(
            [&taken]
            {
                const described_arm described(2, "taken");
                taken = described.directory;

                std::error_code ignored;
                std::filesystem::remove_all(taken, ignored);
            }());

    REQUIRE(!std::filesystem::exists(taken));
}

TEST_CASE("the tooling scenario composes the joint control, the pose, the tool, the world object and the view", "[presets][windows]")
{
    const described_arm described(6, "six");
    const written_model tool("praxis_tooling_tool.stl", 0.1f);
    const written_model world("praxis_tooling_world.stl", 0.2f);
    const presets::arm_scenario chosen = carrying(described.where, tool.where, world.where);

    opened_arm built;
    const std::shared_ptr<scene::preset> composed = open_with(built, chosen, manipulator::baseline());

    REQUIRE(composed_windows(composed) == tooling_windows());
    each_window_opens_one_panel(composed);
}

// Both readings are the composition's own: the pose the arm assembled for its tool and the pose it
// assembled for its flange in the same publication, lifted out where the arm reads a position.
TEST_CASE("a tool attached moves the reported tool pose off the flange, and detaching it puts the two back together", "[presets][windows]")
{
    const described_arm described(6, "six");
    const written_model tool("praxis_tooling_attached.stl", 0.1f);
    const written_model world("praxis_tooling_beside.stl", 0.2f);
    const presets::arm_scenario chosen = carrying(described.where, tool.where, world.where);

    opened_arm built;
    read_through.clear();

    const std::shared_ptr<scene::preset> composed = open_with(built, chosen, recording_poses());
    REQUIRE(composed != nullptr);
    REQUIRE(built.loop.drain().has_value());

    const std::pair<transform, transform> attached = last_publication();
    REQUIRE((attached.first - attached.second).norm() > read_back);

    read_through.clear();
    press_at(*composed->windows[2], 0);
    REQUIRE(built.loop.drain().has_value());

    const std::pair<transform, transform> detached = last_publication();
    REQUIRE((detached.first - detached.second).norm() < read_back);
}

// A slot that answers a plausible pose it never computed is caught once, here, and named. The three
// it was not denied are named nowhere, so a decline reporting the wrong one fails rather than reads.
TEST_CASE("a pose transformation left at its default composes no window and is named", "[presets][windows]")
{
    const described_arm described(6, "six");
    const written_model tool("praxis_tooling_unbound.stl", 0.1f);
    const written_model world("praxis_tooling_unbound_world.stl", 0.2f);
    const presets::arm_scenario chosen = carrying(described.where, tool.where, world.where);

    for(const dependency &denied : pose_transformations)
    {
        INFO(denied.named);

        opened_arm built;
        std::string reported;
        std::shared_ptr<scene::preset> composed;
        {
            const tests::captured_log captured;
            composed = open_with(built, chosen, without(manipulator::baseline(), denied.slot));
            reported = captured.text();
        }

        REQUIRE(composed != nullptr);
        REQUIRE(composed->windows.empty());
        REQUIRE(reported.find(denied.named) != std::string::npos);

        for(const dependency &bound : pose_transformations)
            if(bound.slot != denied.slot)
            {
                INFO(bound.named);
                REQUIRE(reported.find(bound.named) == std::string::npos);
            }
    }
}

// The decline is read once, so it carries every name it owes rather than the first it found.
TEST_CASE("a decline names every pose transformation it was denied", "[presets][windows]")
{
    const described_arm described(6, "six");
    const written_model tool("praxis_tooling_none_bound.stl", 0.1f);
    const written_model world("praxis_tooling_none_bound_world.stl", 0.2f);
    const presets::arm_scenario chosen = carrying(described.where, tool.where, world.where);

    manipulator::capabilities none_of_them = manipulator::baseline();
    for(const dependency &denied : pose_transformations)
        none_of_them = without(none_of_them, denied.slot);

    opened_arm built;
    std::string reported;
    std::shared_ptr<scene::preset> composed;
    {
        const tests::captured_log captured;
        composed = open_with(built, chosen, none_of_them);
        reported = captured.text();
    }

    REQUIRE(composed != nullptr);
    REQUIRE(composed->windows.empty());
    for(const dependency &denied : pose_transformations)
    {
        INFO(denied.named);
        REQUIRE(reported.find(denied.named) != std::string::npos);
    }
}

// The models are loaded and installed before the composer is called, and the decline is reached at
// run time after that. A composition offering no window able to hide them has to take them back, or
// a tool and a world object stand in the scene with nothing anywhere that can reach them.
TEST_CASE("a composition that opens no window draws neither of the models it was given", "[presets][windows]")
{
    const described_arm described(6, "six");
    const written_model tool("praxis_tooling_declined.stl", 0.1f);
    const written_model world("praxis_tooling_declined_world.stl", 0.2f);
    const presets::arm_scenario chosen = carrying(described.where, tool.where, world.where);

    opened_arm built;
    const std::size_t before = built.descendants();

    const std::shared_ptr<scene::preset> composed = open_with(built, chosen, without(manipulator::baseline(), manipulator::robot_slot::position_from_pose));
    REQUIRE(composed != nullptr);
    REQUIRE(composed->windows.empty());

    const auto stencil = std::dynamic_pointer_cast<manipulator::loadable_robot_stencil>(composed->stencil);
    REQUIRE(stencil != nullptr);
    CHECK(stencil->attached_at(manipulator::flange_attachment::tool) == nullptr);
    CHECK(stencil->world_object() == nullptr);

    // The arm itself is not one of the models, and a decline is not a composition that failed: it
    // is still shown, and taking the two models back is all the refusal path does to the scene.
    CHECK(built.descendants() > before);
}

// A capability refuses where it is asked for. This scenario asks for no solve, so a composition that
// binds none opens exactly as one that binds both does.
TEST_CASE("an unbound solve leaves the tooling scenario composing every window it names", "[presets][windows]")
{
    const described_arm described(6, "six");
    const written_model tool("praxis_tooling_unsolved.stl", 0.1f);
    const written_model world("praxis_tooling_unsolved_world.stl", 0.2f);
    const presets::arm_scenario chosen = carrying(described.where, tool.where, world.where);

    manipulator::capabilities without_a_solve = manipulator::baseline();
    without_a_solve.robot.ik_solve_pose       = &manipulator::inert::ik_solve_pose;

    opened_arm built;
    REQUIRE(composed_windows(open_with(built, chosen, without_a_solve)) == tooling_windows());
}

// One object, which is what the attached-models pair and the window between them already carry: a
// second model loaded stands where the first stood rather than beside it.
TEST_CASE("the tooling scenario carries one world object, and a second model replaces it", "[presets][windows]")
{
    const described_arm described(6, "six");
    const written_model tool("praxis_tooling_one_tool.stl", 0.1f);
    const written_model world("praxis_tooling_one_world.stl", 0.2f);
    const written_model instead("praxis_tooling_other_world.stl", 0.4f);
    const presets::arm_scenario chosen = carrying(described.where, tool.where, world.where);

    opened_arm built;
    const std::shared_ptr<scene::preset> composed = open_with(built, chosen, manipulator::baseline());
    REQUIRE(composed != nullptr);

    const auto stencil = std::dynamic_pointer_cast<manipulator::loadable_robot_stencil>(composed->stencil);
    REQUIRE(stencil != nullptr);

    const std::shared_ptr<threepp::Object3D> first = stencil->world_object();
    REQUIRE(first != nullptr);

    scene::imgui_window &panel = *composed->windows[3];
    const std::size_t carried  = built.descendants();

    press_at(panel, 0);
    REQUIRE(stencil->world_object() == nullptr);
    REQUIRE(built.descendants() + 1u == carried);

    type_at(panel, 1, instead.where.string().c_str());
    press_at(panel, 2);

    const std::shared_ptr<threepp::Object3D> second = stencil->world_object();
    REQUIRE(second != nullptr);
    REQUIRE(second != first);
    REQUIRE(built.descendants() == carried);
}

TEST_CASE("the tooling scenario leaves the scene as it found it", "[presets][windows]")
{
    const described_arm described(6, "six");
    const written_model tool("praxis_tooling_released_tool.stl", 0.1f);
    const written_model world("praxis_tooling_released_world.stl", 0.2f);
    const presets::arm_scenario chosen = carrying(described.where, tool.where, world.where);

    opened_arm built;
    const std::size_t before = built.descendants();

    std::shared_ptr<scene::preset> composed = open_with(built, chosen, manipulator::baseline());
    REQUIRE(built.descendants() > before);

    built.release(std::move(composed));
    REQUIRE(built.descendants() == before);
}

// The cases above are over descriptions written for them. This one is over the two machines the
// demonstration actually offers, each carrying six axes and its own meshes.
TEST_CASE("both deployed machines open the tooling scenario", "[presets][windows]")
{
    for(const auto &named : {std::pair<const char *, bool>{"ur_description/urdf/ur.urdf.xacro", true}, std::pair<const char *, bool>{"kuka_kr6_support/urdf/kr6r900sixx.xacro", false}})
    {
        INFO(named.first);

        const std::optional<presets::arm_scenario> chosen = deployed_machine(named.first, named.second);
        if(!chosen)
            SKIP("no robot description is deployed for this configuration");

        opened_arm built;
        REQUIRE(composed_windows(open_with(built, *chosen, manipulator::baseline())) == tooling_windows());
        REQUIRE(drawn_axes(*built.scene) == 6u);
    }
}
