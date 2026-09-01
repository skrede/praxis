#include "fixtures.h"

#include "imgui_frame.h"

#include "praxis/manipulator/arm_state.h"
#include "praxis/manipulator/tool_window.h"
#include "praxis/manipulator/world_object_window.h"
#include "praxis/manipulator/loadable_robot_stencil.h"

#include "praxis/scene/widgets.h"

#include "praxis/scheduler/scheduler.h"

#include "praxis/rigid_motion/capabilities.h"

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

#include <threepp/scenes/Scene.hpp>

#include <imgui.h>

#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <functional>

using namespace praxis::tests;
using namespace praxis::fixture;
using namespace praxis::scheduler;
using namespace praxis::manipulator;

namespace {

const praxis::rigid_motion::frame_ops reference = praxis::rigid_motion::baseline().frame;

const char *const absent_tool_line = "No tool model is attached.";

struct staged
{
    std::shared_ptr<arm_publisher> published;
    std::shared_ptr<threepp::Scene> target;
    std::shared_ptr<loadable_robot_stencil> shown;
};

staged compose(scheduler &loop)
{
    const auto published                         = std::make_shared<arm_publisher>();
    const std::shared_ptr<threepp::Scene> target = threepp::Scene::create();

    return staged{published, target,
                  std::make_shared<loadable_robot_stencil>(two_joint_handle(), attached_models{}, *target, loop.main_strand(), published->reader(),
                                                           praxis::rigid_motion::baseline().screw, praxis::rigid_motion::screw_slot_set{})};
}

using drawing = std::function<void()>;

void tap(imgui_frame &frames, const drawing &draw, ImGuiKey key)
{
    ImGui::GetIO().AddKeyEvent(key, true);
    frames.draw_frame(draw);
    ImGui::GetIO().AddKeyEvent(key, false);
    frames.draw_frame(draw);
}

// The first key a focused panel is given only raises the keyboard cursor, so the absolute step is
// taken twice: the second lands where the first would have whether or not the first was spent.
void reach(imgui_frame &frames, const drawing &draw, ImGuiKey step)
{
    ImGui::GetIO().ConfigFlags |= ImGuiConfigFlags_NavEnableKeyboard;
    frames.draw(
            [&draw]
            {
                ImGui::SetNextWindowFocus();
                draw();
            });
    tap(frames, draw, step);
    tap(frames, draw, step);
}

// The cycle opens on the entry it holds, which on a window holding no tool is the loader, and the
// graphics entry stands one step above it.
void take_graphics_view(imgui_frame &frames, const drawing &draw)
{
    reach(frames, draw, ImGuiKey_Home);
    tap(frames, draw, ImGuiKey_Space);
    tap(frames, draw, ImGuiKey_UpArrow);
    tap(frames, draw, ImGuiKey_Space);
}

// A pane's apply control is the last thing it draws, so the end key reaches its row without counting
// the inputs above it and the sideways steps settle on the rightmost control standing there.
void press_last_control(imgui_frame &frames, const drawing &draw)
{
    reach(frames, draw, ImGuiKey_End);
    tap(frames, draw, ImGuiKey_RightArrow);
    tap(frames, draw, ImGuiKey_RightArrow);
    tap(frames, draw, ImGuiKey_Space);
}

int geometry_of(const drawing &draw)
{
    imgui_frame frames;
    frames.draw(draw);

    REQUIRE(frames.has_draw_data());

    return frames.vertices();
}

// What a window holding no tool draws on that pane is the view cycle with one sentence under it, so
// the geometry of two frames is the whole comparison.
int stating_absent_tool()
{
    return geometry_of(
            []
            {
                int standing = 1;
                const char *const labels[3]{"Kinematics transform", "Graphics transform", "Load .stl"};
                ImGui::Begin("Tool settings");
                ImGui::Combo("Tool view", &standing, labels, 3);
                ImGui::TextUnformatted(absent_tool_line);
                ImGui::End();
            });
}

// The stencil keeps an attachment's offset to itself and lays it onto the node when it renders, so a
// drawn frame is what an assignment that ran would have to show through.
void draw_frame(scheduler &loop, loadable_robot_stencil &shown)
{
    REQUIRE(loop.main_strand().post([&shown] { shown.render(); }).has_value());
    REQUIRE(loop.drain().has_value());
}

}

TEST_CASE("a tool window holding no tool states that on its graphics pane instead of offering to apply one", "[manipulator][tool]")
{
    scheduler loop(inline_workers);
    staged bare = compose(loop);
    tool_window panel("Tool settings", *bare.shown, bare.published->reader(), std::weak_ptr<owned_arm>(), reference);
    panel.initialize();

    draw_frame(loop, *bare.shown);
    REQUIRE(bare.shown->attached_at(flange_attachment::tool) == nullptr);

    const drawing draw = [&panel] { panel.render(); };
    {
        imgui_frame frames;
        take_graphics_view(frames, draw);
        press_last_control(frames, draw);
        CHECK(frames.has_draw_data());
    }

    REQUIRE(panel.state().selected_view == tool_window::tool_view::graphics_transform);
    CHECK(geometry_of(draw) == stating_absent_tool());

    draw_frame(loop, *bare.shown);
    CHECK(bare.shown->attached_at(flange_attachment::tool) == nullptr);
}

TEST_CASE("a world object window holding no object leaves its transform pane with nothing to assign to", "[manipulator][world]")
{
    scheduler loop(inline_workers);
    const Eigen::Vector3f none = Eigen::Vector3f::Zero();
    const world_object_window::settings transforming{false, "", world_object_window::world_view::transform, Eigen::Vector3f::Ones(), none, none};

    staged bare = compose(loop);
    world_object_window loose("World object", *bare.shown, reference, transforming, "machine/world");
    {
        imgui_frame frames;
        press_last_control(frames, [&loose] { loose.render(); });
        CHECK(frames.has_draw_data());
    }

    CHECK(loose.state().gfx_scale.z() > 1.f);
    CHECK(bare.shown->world_object() == nullptr);
}

TEST_CASE("a dropdown handed a selection past the end of a shrunken entry list previews nothing", "[scene][widgets]")
{
    std::vector<std::string> entries{"alpha", "bravo", "charlie", "delta"};
    std::size_t selected   = 3;
    const drawing dropping = [&entries, &selected]
    {
        ImGui::Begin("Presets");
        praxis::scene::render_dropdown_selection("Entry", selected, entries);
        ImGui::End();
    };

    imgui_frame frames;
    frames.draw(dropping);
    const int previewing = frames.vertices();

    entries.resize(1);
    frames.draw(dropping);

    CHECK(frames.has_draw_data());
    CHECK(selected == 3);
    CHECK(frames.vertices() < previewing);
}
