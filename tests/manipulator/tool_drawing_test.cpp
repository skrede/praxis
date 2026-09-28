#include "fixtures.h"
#include "window_stage.h"

#include "../presets/drawn_lines.h"

#include "praxis/manipulator/robot_view_window.h"
#include "praxis/manipulator/loadable_robot_stencil.h"

#include "praxis/rigid_motion/capabilities.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/objects/Mesh.hpp>

#include <threepp/scenes/Scene.hpp>

#include <threepp/geometries/BoxGeometry.hpp>

#include <imgui.h>

#include <Eigen/Core>

#include <array>
#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <optional>

using namespace praxis;
using namespace praxis::tests;
using namespace praxis::fixture;
using namespace praxis::scheduler;
using namespace praxis::manipulator;

namespace {

using controls = robot_view_window::controls;
using opening  = robot_view_window::settings;

constexpr const char *panel_title = "Arm view";

// The rows a window offered the model and tool controls and nothing else draws them on.
constexpr int model_row = 0;
constexpr int tool_row  = 1;

praxis::transform translated(double x, double y, double z)
{
    praxis::transform offset = praxis::transform::Identity();
    offset.block<3, 1>(0, 3) = Eigen::Vector3d(x, y, z);

    return offset;
}

std::shared_ptr<const arm_snapshot> carrying(const praxis::transform &offset)
{
    arm_snapshot seen = at_rest(configuration(0.0, 0.0), Eigen::Vector3d(Eigen::Vector3d::Zero()), praxis::rotation(praxis::rotation::Identity()));
    seen.tool_offset  = offset;

    return std::make_shared<const arm_snapshot>(seen);
}

std::shared_ptr<threepp::Object3D> box()
{
    return threepp::Mesh::create(threepp::BoxGeometry::create(0.05f, 0.05f, 0.05f));
}

std::vector<praxis::screw_axis> two_axes()
{
    const auto revolute = [](double x)
    { return rigid_motion::baseline().screw.screw_axis_from_point_direction_pitch(Eigen::Vector3d(x, 0.0, 0.0), Eigen::Vector3d::UnitZ(), 0.0).value(); };

    return {revolute(0.0), revolute(static_cast<double>(link_length))};
}

// The stencil is told screws, so the arm carries a chain whose switch the model control owns.
struct stage
{
    explicit stage(const praxis::transform &offset = translated(0.1, 0.05, 0.2))
            : loop(inline_workers)
            , scene(threepp::Scene::create())
            , published(std::make_shared<arm_publisher>())
            , shown(two_joint_handle(), attachments{}, *scene, loop.main_strand(), published->reader(), rigid_motion::baseline().screw, rigid_motion::screw_slot_set{})
    {
        published->publish(carrying(offset));
        REQUIRE(shown.initialize().has_value());
        REQUIRE(shown.set_joint_screws(praxis::transform::Identity(), two_axes()).has_value());
        shown.set_flange_attachment(flange_attachment::tool_frame_marker, make_flange_marker(shown.robot()));
        shown.set_flange_attachment(flange_attachment::tool, box());
    }

    void draw()
    {
        REQUIRE(loop.main_strand().post([this] { shown.render(); }).has_value());
        REQUIRE(loop.drain().has_value());
        scene->updateMatrixWorld(true);
    }

    threepp::Object3D *stick()
    {
        return scene->getObjectByName<threepp::Object3D>(loadable_robot_stencil::tool_stick_name());
    }

    threepp::Object3D *mesh()
    {
        return shown.attached_at(flange_attachment::tool).get();
    }

    threepp::Object3D *chain()
    {
        return scene->getObjectByName<threepp::Object3D>(loadable_robot_stencil::chain_name());
    }

    praxis::scheduler::scheduler loop;
    std::shared_ptr<threepp::Scene> scene;
    std::shared_ptr<arm_publisher> published;
    loadable_robot_stencil shown;
};

controls offering(bool model, bool tool)
{
    controls offered;
    offered.model      = model;
    offered.decoration = false;
    offered.tool       = tool;

    return offered;
}

// The list opens its keyboard cursor on its first entry, so the entry is a position in the list. The
// keyboard state a frame is left in outlives the frame, so each entry is taken in a context of its own.
void choose_entry(scene::imgui_window &panel, int row, int entry)
{
    imgui_frame frames;
    const drawing draw = [&panel] { panel.render(); };

    reach_top(frames, draw);
    for(int step = 0; step < row; ++step)
        tap(frames, draw, ImGuiKey_DownArrow);
    tap(frames, draw, ImGuiKey_Space);
    for(int step = 0; step < entry; ++step)
        tap(frames, draw, ImGuiKey_DownArrow);
    tap(frames, draw, ImGuiKey_Space);
}

std::size_t offered_controls(scene::imgui_window &panel)
{
    imgui_frame counting;

    return navigable_items(counting, over(panel));
}

struct tool_entry
{
    int entry;
    bool mesh;
    bool stick;
};

constexpr std::array<tool_entry, 4> tool_entries{tool_entry{0, true, false}, tool_entry{1, false, true}, tool_entry{2, true, true}, tool_entry{3, false, false}};

}

// Both tool switches are written on every entry, and the arm's meshes and chain are moved once, by
// the model control, before the tool control is walked; the model control is then walked back over
// the tool's drawings.
TEST_CASE("each tool entry leaves exactly the drawing it names and the arm's drawings where the model control left them", "[manipulator][controls]")
{
    stage headless;
    robot_view_window panel(panel_title, headless.shown, offering(true, true), opening{});
    panel.initialize();
    choose_entry(panel, model_row, 3);

    for(const tool_entry &chosen : tool_entries)
    {
        INFO("tool entry " << chosen.entry);
        choose_entry(panel, tool_row, chosen.entry);
        headless.draw();
        CHECK(drawn(headless.mesh()) == chosen.mesh);
        CHECK(drawn(headless.stick()) == chosen.stick);
        CHECK_FALSE(drawn(rendered_arm(*headless.scene)));
        CHECK_FALSE(drawn(headless.chain()));
    }

    choose_entry(panel, tool_row, 2);
    choose_entry(panel, model_row, 0);
    headless.draw();
    CHECK(drawn(rendered_arm(*headless.scene)));
    CHECK(drawn(headless.mesh()));
    CHECK(drawn(headless.stick()));
}

TEST_CASE("a stick over a tool offset of the identity is not drawn", "[manipulator][controls]")
{
    stage headless(praxis::transform::Identity());
    robot_view_window panel(panel_title, headless.shown, offering(true, true), opening{});
    panel.initialize();
    choose_entry(panel, tool_row, 1);
    headless.draw();

    REQUIRE(headless.stick() != nullptr);
    CHECK(headless.stick()->parent->visible);
    CHECK_FALSE(drawn(headless.stick()));
}

TEST_CASE("a stick switched off stays hidden however its placement moves it", "[manipulator][controls]")
{
    stage headless;
    robot_view_window panel(panel_title, headless.shown, offering(true, true), opening{});
    panel.initialize();
    choose_entry(panel, tool_row, 1);
    headless.draw();
    REQUIRE(drawn(headless.stick()));

    choose_entry(panel, tool_row, 3);
    headless.published->publish(carrying(translated(-0.2, 0.1, 0.3)));
    headless.draw();

    CHECK(headless.stick()->visible);
    CHECK_FALSE(drawn(headless.stick()));
}

TEST_CASE("a tool mesh attached while the stick alone is chosen is hidden from the next frame", "[manipulator][controls]")
{
    stage headless;
    robot_view_window panel(panel_title, headless.shown, offering(true, true), opening{});
    panel.initialize();
    choose_entry(panel, tool_row, 1);
    headless.draw();

    const std::shared_ptr<threepp::Object3D> later = box();
    headless.shown.set_flange_attachment(flange_attachment::tool, later);
    headless.draw();

    CHECK_FALSE(drawn(later.get()));
    CHECK(drawn(headless.stick()));
}

TEST_CASE("a stencil no view window has touched draws the tool mesh and no stick", "[manipulator][controls]")
{
    stage headless;
    headless.draw();

    REQUIRE(headless.stick() != nullptr);
    CHECK(drawn(headless.mesh()));
    CHECK_FALSE(drawn(headless.stick()));
}

TEST_CASE("a view window not offered the tool control draws one control fewer and still opens the tool as its settings name", "[manipulator][controls]")
{
    stage headless;
    robot_view_window model_alone(panel_title, headless.shown, offering(true, false), opening{});
    robot_view_window model_and_tool(panel_title, headless.shown, offering(true, true), opening{});

    CHECK(offered_controls(model_and_tool) == offered_controls(model_alone) + 1u);

    robot_view_window unoffered(panel_title, headless.shown, offering(false, false), opening{model_render::meshes, true, std::nullopt, true, false, 1.0, tool_render::stick});
    unoffered.initialize();
    headless.draw();

    CHECK(drawn(headless.stick()));
    CHECK_FALSE(drawn(headless.mesh()));
}
