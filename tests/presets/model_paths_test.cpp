#include "opened_arm.h"
#include "described_arm.h"
#include "saved_document.h"
#include "composed_panels.h"
#include "scratch_directory.h"

#include "praxis/presets/arm.h"
#include "praxis/presets/arm_registration.h"

#include "praxis/manipulator/loadable_robot_stencil.h"

#include "praxis/scene/preset.h"
#include "praxis/scene/imgui_window.h"

#include "praxis/config/store.h"
#include "praxis/config/writer.h"
#include "praxis/config/document.h"

#include <catch2/catch_test_macros.hpp>

#include <threepp/core/Object3D.hpp>

#include <memory>
#include <string>
#include <vector>
#include <fstream>
#include <iterator>
#include <filesystem>

using namespace praxis;

namespace {

constexpr const char *tooling = "tool and world object";

std::filesystem::path scratch(const char *named)
{
    const std::filesystem::path directory = fixture::shared_scratch_directory() / named;
    std::filesystem::create_directories(directory);

    return directory;
}

std::filesystem::path holding_models(const char *named)
{
    const std::filesystem::path root = scratch(named);
    fixture::write_model(root / "models" / "tool.stl");
    fixture::write_model(root / "models" / "world.stl");

    return root;
}

config::location tooling_document(const char *named, const std::string &tool, const std::string &world)
{
    const std::string models = "<tool active=\"true\" model=\"" + tool + "\" view=\"kinematics_transform\"/><world_object active=\"true\" model=\"" + world + "\" view=\"transform\"/>";

    return fixture::arm_document(scratch(named), "tooled.xml", fixture::arm_body(tooling, models));
}

std::shared_ptr<scene::preset> opened(fixture::opened_arm &stage, const config::document &carried, const std::vector<std::filesystem::path> &roots)
{
    const presets::arm_scenario chosen      = presets::read_arm(carried, roots);
    std::shared_ptr<scene::preset> composed = stage.open(chosen, presets::arm_windows_tooling(chosen));
    REQUIRE(composed != nullptr);
    REQUIRE(stage.loop.drain().has_value());

    return composed;
}

std::string bytes_of(const config::location &at)
{
    std::ifstream in(at.resolved, std::ios::binary);

    return std::string(std::istreambuf_iterator<char>(in), std::istreambuf_iterator<char>());
}

std::shared_ptr<manipulator::loadable_robot_stencil> stencil_of(const scene::preset &composed)
{
    std::shared_ptr<manipulator::loadable_robot_stencil> shown = std::dynamic_pointer_cast<manipulator::loadable_robot_stencil>(composed.stencil);
    REQUIRE(shown != nullptr);

    return shown;
}

bool draws_both(const scene::preset &composed)
{
    return stencil_of(composed)->attached_at(manipulator::flange_attachment::tool) != nullptr && stencil_of(composed)->world_object() != nullptr;
}

// The flange frame checkbox touches neither model, so what a save of it alone writes shows what a save
// does to models nobody changed.
config::document saved_after_unrelated_edit(const std::shared_ptr<scene::preset> &composed, const config::document &carried, const config::location &at)
{
    fixture::press_at(*fixture::panel_named(composed, "View"), 2);
    const std::vector<config::edit> offered = fixture::offered_by(*composed, carried);
    REQUIRE(offered.size() == 1u);
    REQUIRE(offered.front().key == "robot_view/flange_frame");

    return fixture::saved_into(at, offered);
}

void load_typed_world_object(const std::shared_ptr<scene::preset> &composed, const char *typed)
{
    scene::imgui_window &panel = *fixture::panel_named(composed, "World object");
    fixture::press_at(panel, 0);
    fixture::type_at(panel, 1, typed);
    fixture::press_at(panel, 2);
}

}

TEST_CASE("a tooling document naming its models under a search root opens drawing both and a save keeps both names as written", "[presets][documents]")
{
    const fixture::described_arm described(6, "six");
    const std::filesystem::path models = holding_models("named_models_root");
    const config::location at          = tooling_document("named_models_document", "models/tool.stl", "models/world.stl");
    const config::document carried     = fixture::read_arm_document(at);

    fixture::opened_arm stage;
    const std::shared_ptr<scene::preset> composed = opened(stage, carried, {described.directory, models});
    REQUIRE(draws_both(*composed));
    REQUIRE(fixture::offered_by(*composed, carried).empty());

    const config::document reread = saved_after_unrelated_edit(composed, carried, at);
    CHECK(fixture::text_in(reread, "tool/model") == "models/tool.stl");
    CHECK(fixture::text_in(reread, "world_object/model") == "models/world.stl");
    CHECK(bytes_of(at).find("model=\"models/tool.stl\"") != std::string::npos);
    CHECK(bytes_of(at).find("model=\"models/world.stl\"") != std::string::npos);
}

TEST_CASE("a model typed into the world object window is looked for under the search roots and saved as typed", "[presets][documents]")
{
    const fixture::described_arm described(6, "six");
    const std::filesystem::path models = holding_models("typed_models_root");
    fixture::write_model(models / "models" / "other.stl");
    const config::location at      = tooling_document("typed_models_document", "models/tool.stl", "models/world.stl");
    const config::document carried = fixture::read_arm_document(at);

    fixture::opened_arm stage;
    const std::shared_ptr<scene::preset> composed  = opened(stage, carried, {described.directory, models});
    const std::shared_ptr<threepp::Object3D> first = stencil_of(*composed)->world_object();
    load_typed_world_object(composed, "models/other.stl");
    REQUIRE(stencil_of(*composed)->world_object() != nullptr);
    REQUIRE(stencil_of(*composed)->world_object() != first);

    const std::vector<config::edit> offered = fixture::offered_by(*composed, carried);
    REQUIRE(offered.size() == 1u);
    CHECK(offered.front().key == "world_object/model");
    CHECK(offered.front().value == "models/other.stl");

    const config::document reread = fixture::saved_into(at, offered);
    CHECK(fixture::text_in(reread, "world_object/model") == "models/other.stl");
    CHECK(fixture::text_in(reread, "tool/model") == "models/tool.stl");
}

TEST_CASE("a model path already absolute loads where it stands and is saved back unchanged", "[presets][documents]")
{
    const fixture::described_arm described(6, "six");
    const std::filesystem::path models = holding_models("absolute_models_root");
    const std::string tool             = (models / "models" / "tool.stl").string();
    const std::string world            = (models / "models" / "world.stl").string();
    const config::location at          = tooling_document("absolute_models_document", tool, world);
    const config::document carried     = fixture::read_arm_document(at);

    fixture::opened_arm stage;
    const std::shared_ptr<scene::preset> composed = opened(stage, carried, {described.directory});
    REQUIRE(draws_both(*composed));
    REQUIRE(fixture::offered_by(*composed, carried).empty());

    const config::document reread = saved_after_unrelated_edit(composed, carried, at);
    CHECK(fixture::text_in(reread, "tool/model") == tool);
    CHECK(fixture::text_in(reread, "world_object/model") == world);
    CHECK(bytes_of(at).find("model=\"" + tool + "\"") != std::string::npos);
    CHECK(bytes_of(at).find("model=\"" + world + "\"") != std::string::npos);
}
