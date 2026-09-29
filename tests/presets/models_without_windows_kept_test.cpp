#include "opened_arm.h"
#include "captured_log.h"
#include "carried_models.h"
#include "saved_document.h"
#include "composed_panels.h"
#include "scratch_directory.h"
#include "models_without_windows.h"

#include "praxis/presets/arm.h"

#include "praxis/manipulator/tool_window.h"
#include "praxis/manipulator/tool_configuration.h"
#include "praxis/manipulator/world_object_window.h"

#include "praxis/rigid_motion/frame.h"
#include "praxis/rigid_motion/axis_order.h"
#include "praxis/rigid_motion/capabilities.h"

#include "praxis/config/store.h"
#include "praxis/config/writer.h"
#include "praxis/config/document.h"

#include "praxis/scene/preset.h"

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

#include <limits>
#include <memory>
#include <string>
#include <vector>
#include <sstream>
#include <algorithm>
#include <filesystem>

using namespace praxis;
using namespace praxis::fixture;

namespace {

const char *const rotation_slot       = "frame.rotation_matrix_from_euler";
const char *const transformation_slot = "frame.transformation_matrix_from_rotation_position";

std::string vector_element(const char *named, const char *x, const char *y, const char *z)
{
    return std::string("<") + named + " x=\"" + x + "\" y=\"" + y + "\" z=\"" + z + "\"/>";
}

config::location placed_document(const char *named, const std::filesystem::path &tool, const std::filesystem::path &world)
{
    const std::filesystem::path directory = shared_scratch_directory() / named;
    std::filesystem::create_directories(directory);

    const std::string graphics = "<graphics euler_order=\"XYZ\">" + vector_element("euler", "10", "20", "30") + vector_element("scale", "1.5", "1.5", "1.5") +
            vector_element("offset", "0.05", "-0.02", "0.04") + "</graphics>";
    const std::string kinematics = "<kinematics euler_order=\"ZYX\">" + vector_element("euler", "0", "15", "0") + vector_element("offset", "0.1", "0.2", "0.3") + "</kinematics>";
    const std::string placed     = "<tool active=\"true\" model=\"" + tool.string() + "\" view=\"graphics_transform\">" + graphics + kinematics +
            "</tool><world_object active=\"true\" model=\"" + world.string() + "\" view=\"transform\">" + vector_element("scale", "2", "2", "2") +
            vector_element("offset", "0.4", "-0.3", "0.2") + vector_element("euler_zyx", "15", "25", "35") + "</world_object>";

    return arm_document(directory, "placed.xml", arm_body("tool and world object", placed));
}

void same_tool(const manipulator::tool_window::settings &read, const manipulator::tool_window::settings &carried)
{
    CHECK(read.active == carried.active);
    CHECK(read.model_path == carried.model_path);
    CHECK(read.selected_view == carried.selected_view);
    CHECK(read.gfx_euler_degrees == carried.gfx_euler_degrees);
    CHECK(read.gfx_euler_order == carried.gfx_euler_order);
    CHECK(read.gfx_scale == carried.gfx_scale);
    CHECK(read.gfx_offset == carried.gfx_offset);
    CHECK(read.kinematics_euler_degrees == carried.kinematics_euler_degrees);
    CHECK(read.kinematics_euler_order == carried.kinematics_euler_order);
    CHECK(read.kinematics_offset == carried.kinematics_offset);
}

void same_world_object(const manipulator::world_object_window::settings &read, const manipulator::world_object_window::settings &carried)
{
    CHECK(read.active == carried.active);
    CHECK(read.model_path == carried.model_path);
    CHECK(read.selected_view == carried.selected_view);
    CHECK(read.gfx_scale == carried.gfx_scale);
    CHECK(read.gfx_offset == carried.gfx_offset);
    CHECK(read.gfx_euler_zyx_degrees == carried.gfx_euler_zyx_degrees);
}

rotation unfinished_rotation(const Eigen::Vector3d &, axis_order)
{
    return rotation::Constant(std::numeric_limits<double>::quiet_NaN());
}

std::vector<std::string> lines_of(const std::string &text)
{
    std::vector<std::string> lines;
    std::istringstream read(text);
    for(std::string line; std::getline(read, line);)
        lines.push_back(line);

    return lines;
}

bool some_line_names(const std::vector<std::string> &lines, const std::string &model, const std::string &slot, const std::string &said)
{
    return std::ranges::any_of(lines, [&](const std::string &line)
                               { return line.find(model) != std::string::npos && line.find(slot) != std::string::npos && line.find(said) != std::string::npos; });
}

// What the tooling composition logged while it was opened under `frames`, leaving out the windows
// named in `dropped`.
std::vector<std::string> reported_under(const rigid_motion::frame_ops &frames, std::vector<std::string> dropped)
{
    const described_arm described(6, "six");
    const written_model tool("praxis_reported_tool.stl", 0.1f);
    const written_model world("praxis_reported_world.stl", 0.2f);
    const presets::arm_scenario chosen = placing_models(described.where, tool.where, world.where, true);

    rigid_motion::capabilities motions = rigid_motion::baseline();
    motions.frame                      = frames;

    opened_arm built;
    const tests::captured_log captured;
    const std::shared_ptr<scene::preset> composed = opened_under(built, chosen, leaving_out(presets::arm_windows_tooling(chosen), std::move(dropped)), motions);
    CHECK_FALSE(composed->windows.empty());

    return lines_of(captured.text());
}

rigid_motion::frame_ops frames_with_default_rotation()
{
    rigid_motion::frame_ops frames    = rigid_motion::baseline().frame;
    frames.rotation_matrix_from_euler = &rigid_motion::inert::rotation_matrix_from_euler;

    return frames;
}

}

TEST_CASE("a save from the tooling composition without its tool and world object windows keeps both documents", "[presets][documents]")
{
    const described_arm described(6, "six");
    const written_model tool("praxis_kept_tool.stl", 0.1f);
    const written_model world("praxis_kept_world.stl", 0.2f);
    const config::location at      = placed_document("models_without_windows_kept", tool.where, world.where);
    const config::document carried = read_arm_document(at);

    opened_arm built;
    const std::vector<std::filesystem::path> roots{described.directory};
    const presets::arm_scenario chosen            = presets::read_arm(carried, roots);
    const std::shared_ptr<scene::preset> composed = built.open(chosen, leaving_out(presets::arm_windows_tooling(chosen), {"Tool", "World object"}));
    REQUIRE(built.loop.drain().has_value());

    take_entry_at(*panel_named(composed, "View"), 4, 1);
    const std::vector<config::edit> offered = offered_by(*composed, carried);
    REQUIRE(offered.size() == 1u);
    CHECK(offered.front().key == "robot_view/tool");

    const config::document saved = saved_into(at, offered);
    same_tool(manipulator::read_tool(saved, "tool"), manipulator::read_tool(carried, "tool"));
    same_world_object(manipulator::read_world_object(saved, "world_object"), manipulator::read_world_object(carried, "world_object"));
    CHECK(manipulator::read_tool(carried, "tool").gfx_euler_order == axis_order::xyz);
}

TEST_CASE("a rotation left at its default is named for both models composed without their windows", "[presets][windows]")
{
    const std::vector<std::string> lines = reported_under(frames_with_default_rotation(), {"Tool", "World object"});

    CHECK(some_line_names(lines, "the tool", rotation_slot, "default"));
    CHECK(some_line_names(lines, "the world object", rotation_slot, "default"));
}

TEST_CASE("a frame transformation left at its default is named for the tool alone", "[presets][windows]")
{
    rigid_motion::frame_ops frames                      = rigid_motion::baseline().frame;
    frames.transformation_matrix_from_rotation_position = &rigid_motion::inert::transformation_matrix_from_rotation_position;
    const std::vector<std::string> lines                = reported_under(frames, {"Tool", "World object"});

    CHECK(some_line_names(lines, "the tool", transformation_slot, "default"));
    CHECK_FALSE(some_line_names(lines, "the world object", transformation_slot, ""));
}

TEST_CASE("a rotation answering values that are not finite is named for both models and the transformation is not", "[presets][windows]")
{
    rigid_motion::frame_ops frames       = rigid_motion::baseline().frame;
    frames.rotation_matrix_from_euler    = &unfinished_rotation;
    const std::vector<std::string> lines = reported_under(frames, {"Tool", "World object"});

    CHECK(some_line_names(lines, "the tool", rotation_slot, "not finite"));
    CHECK(some_line_names(lines, "the world object", rotation_slot, "not finite"));
    CHECK_FALSE(some_line_names(lines, "", transformation_slot, ""));
}

TEST_CASE("a rotation left at its default is named with the tool and world object windows composed", "[presets][windows]")
{
    const std::vector<std::string> lines = reported_under(frames_with_default_rotation(), {});

    CHECK(some_line_names(lines, "the tool", rotation_slot, "default"));
    CHECK(some_line_names(lines, "the world object", rotation_slot, "default"));
}
