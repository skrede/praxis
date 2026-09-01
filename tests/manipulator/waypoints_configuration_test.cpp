#include "window_stage.h"
#include "captured_log.h"

#include "praxis/manipulator/waypoints_configuration.h"

#include "praxis/config/error.h"
#include "praxis/config/store.h"
#include "praxis/config/writer.h"
#include "praxis/config/document.h"
#include "praxis/config/declaration.h"

#include "praxis/rigid_motion/capabilities.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <Eigen/Core>

#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <fstream>
#include <algorithm>
#include <filesystem>
#include <string_view>

using namespace praxis;
using namespace praxis::tests;
using namespace praxis::fixture;
using namespace praxis::manipulator;
using Catch::Matchers::ContainsSubstring;

namespace {

constexpr std::size_t joints = 2u;

constexpr std::string_view configurations_at = "machine/joint_waypoints";
constexpr std::string_view poses_at          = "machine/pose_waypoints";

// Both ends carry a configuration in full double precision and a pose in full single precision, so a
// list written and read back names the same rows rather than ones near them.
constexpr double exactly = 0.0;

const rigid_motion::frame_ops frames_of = rigid_motion::baseline().frame;

config::declaration described()
{
    config::declaration shape("probe");
    shape.group("machine");
    declare_joint_waypoints(shape, configurations_at);
    declare_pose_waypoints(shape, poses_at);

    return shape;
}

std::filesystem::path scratch()
{
    const std::filesystem::path directory = std::filesystem::temp_directory_path() / "praxis-waypoints-configuration";
    std::filesystem::create_directories(directory);

    return directory;
}

config::document nothing_carried(const std::string &name)
{
    const std::filesystem::path where = scratch() / name;
    std::filesystem::remove(where);

    return config::load_or_defaults(described(), config::resolve(where, scratch()), config::expectation::partial).values;
}

config::location cleared(const std::string &name)
{
    const std::filesystem::path where = scratch() / name;
    std::filesystem::remove(where);
    REQUIRE(config::write_template(described(), where).has_value());

    return config::resolve(where, scratch());
}

config::document loaded(const config::location &at)
{
    const config::outcome answered = config::load_or_defaults(described(), at);
    INFO((answered.failure ? answered.failure->message : std::string()));
    REQUIRE_FALSE(answered.failure.has_value());

    return answered.values;
}

config::document saved_and_reloaded(const config::location &at, const std::vector<config::edit> &changes)
{
    const expected<void, config::error> saved = config::save(described(), at, changes);
    INFO((saved ? std::string() : saved.error().message));
    REQUIRE(saved.has_value());

    return loaded(at);
}

config::document carrying(const std::string &name, std::string_view body)
{
    const std::filesystem::path where = scratch() / name;
    std::ofstream out(where, std::ios::binary | std::ios::trunc);
    out << "<probe><machine>" << body << "</machine></probe>\n";
    out.close();

    const config::outcome answered = config::load_or_defaults(described(), config::resolve(where, scratch()), config::expectation::partial);
    INFO((answered.failure ? answered.failure->message : std::string()));
    REQUIRE_FALSE(answered.failure.has_value());

    return answered.values;
}

joint_vector row_at(double first, double second)
{
    joint_vector taken(2);
    taken << first, second;

    return taken;
}

joint_waypoint_list::settings three_configurations()
{
    return joint_waypoint_list::settings{{row_at(0.25, -0.5), row_at(1.5, 0.125), row_at(-2.25, 3.0)}};
}

edited_pose pose_at(float x, float first)
{
    edited_pose taken;
    taken.position      = Eigen::Vector3f(x, 0.25f, -0.5f);
    taken.euler_degrees = Eigen::Vector3f(first, 90.f, -180.f);

    return taken;
}

pose_waypoint_list::settings three_poses()
{
    return pose_waypoint_list::settings{{pose_at(0.125f, 10.f), pose_at(0.25f, 20.f), pose_at(-0.5f, 30.f)}};
}

// Every key one group declares beneath the one it hangs under, so what two groups have in common is
// read off the declarations themselves rather than off the paths a caller happened to choose.
std::vector<std::string> declared_by(void (*declaring)(config::declaration &, std::string_view), std::string_view at)
{
    config::declaration shape("probe");
    shape.group("machine");
    declaring(shape, at);

    std::vector<std::string> keys;
    for(const config::node &named : shape.nodes())
        if(named.path != "machine")
            keys.push_back(named.path);

    return keys;
}

void stands_at(const joint_vector &read, const joint_vector &written)
{
    REQUIRE(read.size() == written.size());
    CHECK(read.isApprox(written, exactly));
}

void stands_at(const edited_pose &read, const edited_pose &written)
{
    CHECK(read.position == written.position);
    CHECK(read.euler_degrees == written.euler_degrees);
}

std::shared_ptr<arm_publisher> published_arm()
{
    return publishing(at_rest(configuration(0.0, 0.0), Eigen::Vector3d(0.4, -0.2, 0.3), rotation::Identity()));
}

}

TEST_CASE("a document naming no row at all yields both lists carrying none", "[manipulator][waypoints]")
{
    const config::document carried = nothing_carried("absent.xml");

    CHECK(read_joint_waypoints(carried, configurations_at, joints).rows.empty());
    CHECK(read_pose_waypoints(carried, poses_at).rows.empty());
}

TEST_CASE("a document carrying a list of configurations yields them in the order it carries them", "[manipulator][waypoints]")
{
    const config::document carried = carrying("authored-configurations.xml",
                                              "<joint_waypoints><waypoint><index>1</index><joints>0.25 -0.5</joints></waypoint>"
                                              "<waypoint><index>2</index><joints>1.5 0.125</joints></waypoint>"
                                              "<waypoint><index>3</index><joints>-2.25 3</joints></waypoint></joint_waypoints>");

    const joint_waypoint_list::settings read = read_joint_waypoints(carried, configurations_at, joints);

    REQUIRE(read.rows.size() == 3u);
    stands_at(read.rows[0], row_at(0.25, -0.5));
    stands_at(read.rows[1], row_at(1.5, 0.125));
    stands_at(read.rows[2], row_at(-2.25, 3.0));
}

TEST_CASE("a document carrying a list of poses yields six numbers a row in the order it carries them", "[manipulator][waypoints]")
{
    const config::document carried = carrying("authored-poses.xml",
                                              "<pose_waypoints><waypoint><index>1</index><pose>0.125 0.25 -0.5 10 90 -180</pose></waypoint>"
                                              "<waypoint><index>2</index><pose>0.25 0.25 -0.5 20 90 -180</pose></waypoint></pose_waypoints>");

    const pose_waypoint_list::settings read = read_pose_waypoints(carried, poses_at);

    REQUIRE(read.rows.size() == 2u);
    stands_at(read.rows[0], pose_at(0.125f, 10.f));
    stands_at(read.rows[1], pose_at(0.25f, 20.f));
}

TEST_CASE("a list of configuration waypoints written through the declared keys reads back as it was set", "[manipulator][waypoints]")
{
    const config::location at                   = cleared("configurations.xml");
    const joint_waypoint_list::settings written = three_configurations();
    const config::document carried              = saved_and_reloaded(at, write_joint_waypoints(loaded(at), written, configurations_at));

    const joint_waypoint_list::settings read = read_joint_waypoints(carried, configurations_at, joints);

    REQUIRE(read.rows.size() == written.rows.size());
    for(std::size_t row = 0; row < read.rows.size(); ++row)
        stands_at(read.rows[row], written.rows[row]);
}

TEST_CASE("a list of pose waypoints written through the declared keys reads back as it was set", "[manipulator][waypoints]")
{
    const config::location at                  = cleared("poses.xml");
    const pose_waypoint_list::settings written = three_poses();
    const config::document carried             = saved_and_reloaded(at, write_pose_waypoints(loaded(at), written, poses_at));

    const pose_waypoint_list::settings read = read_pose_waypoints(carried, poses_at);

    REQUIRE(read.rows.size() == written.rows.size());
    for(std::size_t row = 0; row < read.rows.size(); ++row)
        stands_at(read.rows[row], written.rows[row]);
}

// A document carries no way to drop a row, so what a shorter list does to the rows it no longer
// reaches is the whole of what "saved shorter" means.
TEST_CASE("a shorter list saved over a longer one empties the rows it no longer reaches", "[manipulator][waypoints]")
{
    const config::location at    = cleared("shortened.xml");
    const config::document three = saved_and_reloaded(at, write_joint_waypoints(loaded(at), three_configurations(), configurations_at));
    const joint_waypoint_list::settings one{{row_at(1.5, 0.125)}};
    const config::document carried = saved_and_reloaded(at, write_joint_waypoints(three, one, configurations_at));

    const joint_waypoint_list::settings read = read_joint_waypoints(carried, configurations_at, joints);

    REQUIRE(read.rows.size() == 1u);
    stands_at(read.rows.front(), row_at(1.5, 0.125));
}

TEST_CASE("a row of the wrong width in the document is named through the log and the rows beside it are still read", "[manipulator][waypoints]")
{
    const config::document carried = carrying("wide-row.xml",
                                              "<joint_waypoints><waypoint><index>1</index><joints>0.25 -0.5</joints></waypoint>"
                                              "<waypoint><index>2</index><joints>1.5 0.125 3</joints></waypoint>"
                                              "<waypoint><index>3</index><joints>-2.25 3</joints></waypoint></joint_waypoints>");

    joint_waypoint_list::settings read;
    const std::string said = reported_by([&carried, &read] { read = read_joint_waypoints(carried, configurations_at, joints); });

    REQUIRE(read.rows.size() == 2u);
    stands_at(read.rows[0], row_at(0.25, -0.5));
    stands_at(read.rows[1], row_at(-2.25, 3.0));
    CHECK_THAT(said, ContainsSubstring("manipulator.read_joint_waypoints"));
    CHECK_THAT(said, ContainsSubstring("a row of 3 joint values at row 2 for an arm of 2 joints"));
    CHECK_THAT(said, ContainsSubstring("the rows beside it still are"));
}

TEST_CASE("a pose row carrying other than six numbers is named through the log and the rows beside it are still read", "[manipulator][waypoints]")
{
    const config::document carried = carrying("short-pose.xml",
                                              "<pose_waypoints><waypoint><index>1</index><pose>0.125 0.25 -0.5 10 90 -180</pose></waypoint>"
                                              "<waypoint><index>2</index><pose>0.25 0.25 -0.5</pose></waypoint>"
                                              "<waypoint><index>3</index><pose>0.25 0.25 -0.5 20 90 -180</pose></waypoint></pose_waypoints>");

    pose_waypoint_list::settings read;
    const std::string said = reported_by([&carried, &read] { read = read_pose_waypoints(carried, poses_at); });

    REQUIRE(read.rows.size() == 2u);
    stands_at(read.rows[0], pose_at(0.125f, 10.f));
    stands_at(read.rows[1], pose_at(0.25f, 20.f));
    CHECK_THAT(said, ContainsSubstring("manipulator.read_pose_waypoints"));
    CHECK_THAT(said, ContainsSubstring("a row of 3 numbers at row 2 where a pose is 6"));
}

// One key path would let a document written by a list of one kind be read by a list of the other, so
// what keeps the two apart is that neither declares a key the other reads.
TEST_CASE("the two waypoint lists declare under keys neither of them shares", "[manipulator][waypoints]")
{
    const std::vector<std::string> configurations = declared_by(&declare_joint_waypoints, configurations_at);
    const std::vector<std::string> poses          = declared_by(&declare_pose_waypoints, poses_at);

    REQUIRE_FALSE(configurations.empty());
    REQUIRE_FALSE(poses.empty());
    for(const std::string &key : configurations)
    {
        INFO(key);
        CHECK(std::find(poses.begin(), poses.end(), key) == poses.end());
    }
}

// The seam an application calls when it saves is the window's own `settings_edits`, so this drives
// that rather than the free function beneath it.
TEST_CASE("each waypoint list's own edits save and read back as the rows it holds", "[manipulator][waypoints]")
{
    const std::shared_ptr<arm_publisher> published = published_arm();

    const config::location configurations_file = cleared("configurations-window.xml");
    const joint_waypoint_list configurations("Waypoints", published->reader(), frames_of, three_configurations(), std::string(configurations_at));
    const std::vector<config::edit> offered = configurations.settings_edits(loaded(configurations_file));

    REQUIRE_FALSE(offered.empty());
    REQUIRE(read_joint_waypoints(saved_and_reloaded(configurations_file, offered), configurations_at, joints).rows.size() == 3u);

    const config::location poses_file = cleared("poses-window.xml");
    const pose_waypoint_list poses("Poses", published->reader(), frames_of, three_poses(), std::string(poses_at));
    const std::vector<config::edit> posed = poses.settings_edits(loaded(poses_file));

    REQUIRE_FALSE(posed.empty());
    REQUIRE(read_pose_waypoints(saved_and_reloaded(poses_file, posed), poses_at).rows.size() == 3u);
}

// A configuration row is typed in degrees and answered in radians, so a document written from
// anywhere else is not a value the list can offer back unchanged. What a save has to settle at is
// its own output: one save and the document is a fixed point, and a list reopened on it offers
// nothing further.
TEST_CASE("a waypoint list reopened on the document its own save wrote offers no further edit", "[manipulator][waypoints]")
{
    const std::shared_ptr<arm_publisher> published = published_arm();
    const config::location at                      = cleared("unmoved.xml");
    const joint_waypoint_list first("Waypoints", published->reader(), frames_of, three_configurations(), std::string(configurations_at));
    const config::document once = saved_and_reloaded(at, first.settings_edits(loaded(at)));

    const joint_waypoint_list panel("Waypoints", published->reader(), frames_of, read_joint_waypoints(once, configurations_at, joints), std::string(configurations_at));

    CHECK(panel.settings_edits(once).empty());
}

// A pose row is held in the unit and the precision it is typed in from end to end, so a pose list is
// a fixed point on any document, not only on one it wrote.
TEST_CASE("a pose list whose document already reads as its rows offers no edit at all", "[manipulator][waypoints]")
{
    const std::shared_ptr<arm_publisher> published = published_arm();
    const config::location at                      = cleared("pose-unmoved.xml");
    const config::document carried                 = saved_and_reloaded(at, write_pose_waypoints(loaded(at), three_poses(), poses_at));

    const pose_waypoint_list panel("Poses", published->reader(), frames_of, read_pose_waypoints(carried, poses_at), std::string(poses_at));

    CHECK(panel.settings_edits(carried).empty());
}
