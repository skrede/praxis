#include "fixtures.h"
#include "window_stage.h"

#include "imgui_frame.h"
#include "panel_labels.h"

#include "../presets/scratch_directory.h"

#include "praxis/manipulator/arm_state.h"
#include "praxis/manipulator/model_file.h"
#include "praxis/manipulator/tool_window.h"
#include "praxis/manipulator/world_object_window.h"
#include "praxis/manipulator/loadable_robot_stencil.h"

#include "praxis/scene/imgui_window.h"

#include "praxis/scheduler/scheduler.h"

#include "praxis/rigid_motion/capabilities.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/scenes/Scene.hpp>

#include <imgui.h>

#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <fstream>
#include <filesystem>

using namespace praxis::tests;
using namespace praxis::fixture;
using namespace praxis::scheduler;
using namespace praxis::manipulator;

namespace {

const praxis::rigid_motion::frame_ops reference = praxis::rigid_motion::baseline().frame;

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
                  std::make_shared<loadable_robot_stencil>(two_joint_handle(), attachments{}, *target, loop.main_strand(), published->reader(), praxis::rigid_motion::baseline().screw,
                                                           praxis::rigid_motion::screw_slot_set{})};
}

std::filesystem::path scratch_root(const char *named)
{
    const std::filesystem::path root = shared_scratch_directory() / named;
    std::filesystem::create_directories(root / "models");

    return root;
}

std::filesystem::path written_model(const std::filesystem::path &where)
{
    std::ofstream document(where);
    document << "solid praxis\nfacet normal 0 0 1\nouter loop\nvertex 0 0 0\nvertex 0.1 0 0\nvertex 0 0.1 0\nendloop\nendfacet\nendsolid praxis\n";

    return where;
}

// The field and the button are pressed in frames of their own, as a person reaches one and then the
// other.
void type_and_load(praxis::scene::imgui_window &panel, std::size_t field, const char *typed)
{
    const drawing draw = [&panel] { panel.render(); };
    {
        imgui_frame frames;
        frames.assert_on_frame_faults(true);
        type_into(frames, draw, field, typed);
    }

    imgui_frame frames;
    frames.assert_on_frame_faults(true);
    stand_below_top(frames, draw, field + 1u);
    tap(frames, draw, ImGuiKey_Space);
}

// A tool window holding no tool opens at its loader, so what it draws after a load no root could
// answer is the view cycle, the field, the button and the one line saying why nothing loaded.
std::size_t stating_unheld(const std::string &typed)
{
    return geometry_of(
            [&typed]
            {
                int standing = 2;
                char held[1024]{};
                typed.copy(held, sizeof(held) - 1u);
                const char *const labels[3]{"Kinematics transform", "Graphics transform", "Load .stl"};
                ImGui::Begin("Tool settings");
                ImGui::Combo("Tool view", &standing, labels, 3);
                ImGui::InputText("STL file", held, sizeof(held));
                ImGui::Button("Load");
                ImGui::Text("Loaded model: %s", ("none, no search root holds " + typed).c_str());
                ImGui::End();
            });
}

}

TEST_CASE("a tool window loads a model typed relative to its search roots and keeps the path as typed", "[manipulator][tool]")
{
    const std::filesystem::path root = scratch_root("typed_tool_root");
    written_model(root / "models" / "typed.stl");

    scheduler loop(inline_workers);
    staged bare = compose(loop);
    tool_window panel("Tool settings", *bare.shown, bare.published->reader(), std::weak_ptr<owned_arm>(), reference, tool_window::settings{}, std::string(), {root});
    panel.initialize();
    REQUIRE(bare.shown->attached_at(flange_attachment::tool) == nullptr);

    type_and_load(panel, 1u, "models/typed.stl");

    CHECK(bare.shown->attached_at(flange_attachment::tool) != nullptr);
    CHECK(panel.state().model_path == "models/typed.stl");
}

TEST_CASE("a world object window loads a model typed relative to its search roots and keeps the path as typed", "[manipulator][world]")
{
    const std::filesystem::path root = scratch_root("typed_world_root");
    written_model(root / "models" / "typed.stl");

    scheduler loop(inline_workers);
    staged bare = compose(loop);
    world_object_window panel("World object", *bare.shown, reference, world_object_window::settings{}, std::string(), {root});
    panel.initialize();
    REQUIRE(bare.shown->world_object() == nullptr);

    type_and_load(panel, 0u, "models/typed.stl");

    CHECK(bare.shown->world_object() != nullptr);
    CHECK(panel.state().model_path == "models/typed.stl");
}

TEST_CASE("a tool window whose typed model no search root holds says so on its loader pane", "[manipulator][tool]")
{
    const std::filesystem::path root = scratch_root("absent_tool_root");

    scheduler loop(inline_workers);
    staged bare = compose(loop);
    tool_window panel("Tool settings", *bare.shown, bare.published->reader(), std::weak_ptr<owned_arm>(), reference, tool_window::settings{}, std::string(), {root});
    panel.initialize();

    type_and_load(panel, 1u, "models/absent.stl");

    REQUIRE(bare.shown->attached_at(flange_attachment::tool) == nullptr);
    CHECK(panel.state().model_path == "models/absent.stl");
    CHECK(geometry_of([&panel] { panel.render(); }) == stating_unheld("models/absent.stl"));
}

TEST_CASE("a tool window whose Load fails after a model was loaded takes that model off the flange and offers no Active switch", "[manipulator][tool]")
{
    const std::filesystem::path root = scratch_root("replaced_tool_root");
    written_model(root / "models" / "typed.stl");

    scheduler loop(inline_workers);
    staged bare = compose(loop);
    tool_window panel("Tool settings", *bare.shown, bare.published->reader(), std::weak_ptr<owned_arm>(), reference, tool_window::settings{}, std::string(), {root});
    panel.initialize();
    type_and_load(panel, 1u, "models/typed.stl");
    REQUIRE(bare.shown->attached_at(flange_attachment::tool) != nullptr);
    REQUIRE(panel.state().active);

    {
        imgui_frame frames;
        frames.assert_on_frame_faults(true);
        take_entry_on(frames, [&panel] { panel.render(); }, "Tool settings", "Tool view", 2u);
    }
    type_and_load(panel, 2u, "models/absent.stl");

    CHECK(bare.shown->attached_at(flange_attachment::tool) == nullptr);
    CHECK_FALSE(panel.state().active);
    CHECK(panel.state().model_path == "models/absent.stl");
    CHECK(geometry_of([&panel] { panel.render(); }) == stating_unheld("models/absent.stl"));
}

TEST_CASE("a tool window opened at a model no search root holds reports that on its loader pane and holds nothing at the flange", "[manipulator][tool]")
{
    const std::filesystem::path root = scratch_root("opened_absent_tool_root");

    scheduler loop(inline_workers);
    staged bare = compose(loop);
    tool_window panel("Tool settings", *bare.shown, bare.published->reader(), std::weak_ptr<owned_arm>(), reference, tool_window::settings{false, "models/absent.stl"}, std::string(),
                      {root});
    panel.initialize();

    CHECK(bare.shown->attached_at(flange_attachment::tool) == nullptr);
    CHECK(geometry_of([&panel] { panel.render(); }) == stating_unheld("models/absent.stl"));
}

TEST_CASE("a model name is located under the first search root holding it", "[manipulator][model]")
{
    const std::filesystem::path first  = scratch_root("first_holding_root");
    const std::filesystem::path second = scratch_root("second_holding_root");
    const std::filesystem::path bare   = scratch_root("root_holding_nothing");
    written_model(first / "models" / "m.stl");
    written_model(second / "models" / "m.stl");

    const std::vector<std::filesystem::path> both{first, second};
    CHECK(located_model("models/m.stl", both) == first / "models" / "m.stl");

    const std::vector<std::filesystem::path> later{bare, second};
    CHECK(located_model("models/m.stl", later) == second / "models" / "m.stl");
}

TEST_CASE("a model name no root holds is located where it stands and nowhere otherwise", "[manipulator][model]")
{
    const std::filesystem::path standing = written_model(scratch_root("standing_model") / "models" / "standing.stl");
    REQUIRE(standing.is_absolute());

    CHECK(located_model(standing, {}) == standing);
    CHECK_FALSE(located_model("models/nowhere_at_all.stl", {}).has_value());
}

TEST_CASE("an absolute model name is located where it stands whatever the roots", "[manipulator][model]")
{
    const std::filesystem::path standing = written_model(scratch_root("absolute_model") / "models" / "absolute.stl");
    const std::vector<std::filesystem::path> roots{scratch_root("root_beside_absolute")};
    REQUIRE(standing.is_absolute());

    CHECK(located_model(standing, roots) == standing);
}

TEST_CASE("a blank model name and a name naming a directory locate no file", "[manipulator][model]")
{
    const std::vector<std::filesystem::path> roots{scratch_root("root_holding_a_directory")};

    CHECK_FALSE(located_model("", roots).has_value());
    CHECK_FALSE(located_model("models", roots).has_value());
}

TEST_CASE("a located model file loads as a mesh", "[manipulator][model]")
{
    const std::filesystem::path standing = written_model(scratch_root("loaded_model") / "models" / "loaded.stl");

    CHECK(loaded_model(standing) != nullptr);
}
