#include "fixtures.h"
#include "window_stage.h"

#include "praxis/manipulator/edited_pose.h"
#include "praxis/manipulator/configuration.h"
#include "praxis/manipulator/task_space_window.h"

#include "praxis/config/store.h"
#include "praxis/config/writer.h"
#include "praxis/config/document.h"
#include "praxis/config/declaration.h"

#include "praxis/rigid_motion/axis_order.h"
#include "praxis/rigid_motion/capabilities.h"

#include <catch2/catch_test_macros.hpp>

#include <Eigen/Core>

#include <memory>
#include <string>
#include <vector>
#include <fstream>
#include <algorithm>
#include <filesystem>
#include <string_view>

using namespace praxis;
using namespace praxis::fixture;
using namespace praxis::manipulator;

namespace {

constexpr std::string_view pose_at       = "machine/tool_pose";
constexpr std::string_view task_space_at = "machine/task_space";

config::declaration described()
{
    config::declaration shape("probe");
    shape.group("machine");
    declare_task_space(shape, task_space_at);
    declare_shared_pose(shape, pose_at);
    return shape;
}

std::filesystem::path scratch()
{
    const std::filesystem::path directory = std::filesystem::temp_directory_path() / "praxis-shared-pose-configuration";
    std::filesystem::create_directories(directory);
    return directory;
}

// A starter document written from the declaration, with `changes` saved into it.
config::document starter(const std::string &name, const std::vector<config::edit> &changes = {})
{
    const std::filesystem::path where = scratch() / name;
    std::filesystem::remove(where);
    REQUIRE(config::write_template(described(), where).has_value());

    const config::location at_file = config::resolve(where, scratch());
    if(!changes.empty())
        REQUIRE(config::save(described(), at_file, changes).has_value());

    const config::outcome answered = config::load_or_defaults(described(), at_file);
    REQUIRE_FALSE(answered.failure.has_value());
    return answered.values;
}

// `body` is written between the machine element's tags; an empty body is a document carrying no pose.
config::document authored(const std::string &name, const std::string &body)
{
    const std::filesystem::path where = scratch() / name;
    std::ofstream(where, std::ios::binary | std::ios::trunc) << "<probe><machine>" << body << "</machine></probe>\n";

    const config::outcome answered = config::load_or_defaults(described(), config::resolve(where, scratch()), config::expectation::partial);
    INFO((answered.failure ? answered.failure->message : std::string()));
    REQUIRE_FALSE(answered.failure.has_value());
    return answered.values;
}

// Every one of its seven values apart from the declared fallbacks.
edited_pose chosen_pose(pose_standing standing)
{
    edited_pose pose;
    pose.order         = axis_order::xyz;
    pose.standing      = standing;
    pose.position      = Eigen::Vector3f{0.25f, -0.4f, 0.7f};
    pose.euler_degrees = Eigen::Vector3f{37.f, 52.f, -19.f};
    return pose;
}

std::size_t pose_edits(const std::vector<config::edit> &offered)
{
    return static_cast<std::size_t>(std::ranges::count_if(offered, [](const config::edit &one) { return one.key.starts_with(pose_at); }));
}

}

TEST_CASE("a held shared pose written through the declared keys reads back held at every value", "[manipulator][configuration]")
{
    const edited_pose written = chosen_pose(pose_standing::held);
    const edited_pose read    = read_shared_pose(starter("held-pose.xml", write_shared_pose(written, pose_at)), pose_at);

    CHECK(read.standing == pose_standing::held);
    CHECK(read.order == axis_order::xyz);
    CHECK(read.position == written.position);
    CHECK(read.euler_degrees == written.euler_degrees);
}

TEST_CASE("a document carrying any one of the six numbers reads the shared pose held", "[manipulator][configuration]")
{
    for(const std::string vector : {"position", "euler"})
        for(const std::string component : {"x", "y", "z"})
        {
            INFO(vector + "/" + component);
            const std::string body = "<tool_pose><" + vector + " " + component + "=\"0.5\"/></tool_pose>";
            CHECK(read_shared_pose(authored("one-" + vector + "-" + component + ".xml", body), pose_at).standing == pose_standing::held);
        }
}

TEST_CASE("a document naming only the Euler order reads the shared pose unset in that order", "[manipulator][configuration]")
{
    const edited_pose read = read_shared_pose(authored("order-only.xml", "<tool_pose euler_order=\"XYZ\"/>"), pose_at);

    CHECK(read.standing == pose_standing::unset);
    CHECK(read.order == axis_order::xyz);
}

TEST_CASE("a document naming no shared pose reads it unset at the origin", "[manipulator][configuration]")
{
    const edited_pose read = read_shared_pose(authored("no-pose.xml", ""), pose_at);

    CHECK(read.standing == pose_standing::unset);
    CHECK(read.order == axis_order::zyx);
    CHECK(read.position.isZero());
    CHECK(read.euler_degrees.isZero());
}

TEST_CASE("a seeded or provisional or unset pose offers nothing to save", "[manipulator][configuration]")
{
    const config::document carried = starter("unheld-pose.xml");

    for(const pose_standing standing : {pose_standing::seeded, pose_standing::provisional, pose_standing::unset})
        CHECK(unsaved_shared_pose(carried, chosen_pose(standing), pose_at).empty());

    CHECK(unsaved_shared_pose(carried, chosen_pose(pose_standing::held), "").empty());
}

TEST_CASE("a held pose over a document carrying none of it offers all seven values even where they equal the fallbacks", "[manipulator][configuration]")
{
    edited_pose origin;
    origin.standing                         = pose_standing::held;
    const std::vector<config::edit> offered = unsaved_shared_pose(authored("none-carried.xml", ""), origin, pose_at);

    CHECK(offered.size() == 7u);
    CHECK(pose_edits(offered) == 7u);
}

TEST_CASE("a held pose read from a document carrying part of it offers nothing until a value moves", "[manipulator][configuration]")
{
    const config::document carried = authored("part-carried.xml", "<tool_pose><position x=\"0.5\"/></tool_pose>");
    edited_pose read               = read_shared_pose(carried, pose_at);
    REQUIRE(read.standing == pose_standing::held);
    CHECK(unsaved_shared_pose(carried, read, pose_at).empty());

    read.euler_degrees.x()                  = 15.f;
    const std::vector<config::edit> offered = unsaved_shared_pose(carried, read, pose_at);

    REQUIRE(offered.size() == 1u);
    CHECK(offered.front().key == "machine/tool_pose/euler/x");
    CHECK(offered.front().value == "15");
}

TEST_CASE("a starter document written from a declaration carrying the shared pose reads it held at the origin", "[manipulator][configuration]")
{
    const edited_pose read = read_shared_pose(starter("starter-pose.xml"), pose_at);

    CHECK(read.standing == pose_standing::held);
    CHECK(read.order == axis_order::zyx);
    CHECK(read.position.isZero());
    CHECK(read.euler_degrees.isZero());
}

TEST_CASE("a task space window offers the shared pose only under a pose path and only once the pose is held", "[manipulator][configuration]")
{
    const std::shared_ptr<arm_publisher> published = publishing(at_rest(configuration(0.0, 0.0), Eigen::Vector3d::Zero(), rotation::Identity()));
    const rigid_motion::frame_ops reference        = rigid_motion::baseline().frame;
    const auto shared                              = std::make_shared<edited_pose>(chosen_pose(pose_standing::seeded));
    const config::document carried                 = starter("window-pose.xml");
    const task_space_window::settings state;

    const task_space_window posing("Target pose", published->reader(), std::weak_ptr<owned_arm>(), reference, shared, state, std::string(task_space_at), std::string(pose_at));
    const task_space_window pathless("Target pose", published->reader(), std::weak_ptr<owned_arm>(), reference, shared, state, std::string(task_space_at));
    CHECK(posing.settings_edits(carried).empty());

    shared->standing = pose_standing::held;
    CHECK(pose_edits(posing.settings_edits(carried)) == 7u);
    CHECK(posing.settings_edits(carried).size() == 7u);
    CHECK(pose_edits(pathless.settings_edits(carried)) == 0u);
}
