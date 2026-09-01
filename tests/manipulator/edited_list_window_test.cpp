#include "window_stage.h"
#include "captured_log.h"

#include "praxis/manipulator/edited_list_window.h"

#include "praxis/rigid_motion/angles.h"
#include "praxis/rigid_motion/capabilities.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <catch2/matchers/catch_matchers_exception.hpp>

#include <imgui.h>
#include <imgui_internal.h>

#include <Eigen/Core>

#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <numbers>
#include <stdexcept>

using namespace praxis;
using namespace praxis::tests;
using namespace praxis::fixture;
using namespace praxis::manipulator;
using Catch::Matchers::Message;
using Catch::Matchers::ContainsSubstring;

namespace {

// Both ends of a row hold it in single precision, so a value that made the trip is the value behind
// it to within one float step of it.
constexpr double float_step = 1.0e-5;

const Eigen::Vector3d reached(0.4, -0.2, 0.3);
const rotation upright(rotation::Identity());

const rigid_motion::frame_ops frames_of = rigid_motion::baseline().frame;

arm_snapshot two_joint_arm_at_rest()
{
    return at_rest(configuration(0.0, 0.0), reached, upright);
}

arm_snapshot jointless_arm_at_rest()
{
    return at_rest(joint_vector(), reached, upright);
}

arm_snapshot arm_without_a_tool_pose()
{
    return at_rest(configuration(0.0, 0.0), unexpected(refusal::not_implemented), unexpected(refusal::not_implemented));
}

joint_vector row_at(double first, double second)
{
    return configuration(first * radians_per_degree, second * radians_per_degree);
}

std::vector<joint_vector> three_configurations()
{
    return {row_at(10.0, 20.0), row_at(30.0, 40.0), row_at(50.0, 60.0)};
}

edited_pose pose_at(float x, float first)
{
    edited_pose taken;
    taken.position      = Eigen::Vector3f(x, 0.f, 0.5f);
    taken.euler_degrees = Eigen::Vector3f(first, 0.f, 180.f);

    return taken;
}

std::vector<edited_pose> three_poses()
{
    return {pose_at(0.1f, 10.f), pose_at(0.2f, 20.f), pose_at(0.3f, 30.f)};
}

void stands_at(const joint_vector &read, const joint_vector &expected)
{
    REQUIRE(read.size() == expected.size());
    CHECK(read.isApprox(expected, float_step));
}

void stands_at(const edited_pose &read, const edited_pose &expected)
{
    CHECK(read.position.isApprox(expected.position, static_cast<float>(float_step)));
    CHECK(read.euler_degrees.isApprox(expected.euler_degrees, static_cast<float>(float_step)));
}

ImGuiID standing_on()
{
    return ImGui::GetCurrentContext()->NavId;
}

// Steps the way it is given until the cursor stops moving, which is the control at that end of the
// row: a step past either end of a row reaches nothing, so the walk says where the row ends rather
// than a count of what it holds having to.
void walk_to_end(imgui_frame &frames, const drawing &draw, ImGuiKey step)
{
    for(ImGuiID standing = 0; standing != standing_on();)
    {
        standing = standing_on();
        tap(frames, draw, step);
    }
}

// The context one panel is driven over is global state a single pointer reaches, so a case driving
// two panels drives them one after the other and never holds two frames open at once. A step upward
// reaches the row above at whichever of its controls the library scores nearest, so the walk left
// puts the cursor on the first of them whatever that was.
void reach_row(imgui_frame &frames, const drawing &draw, std::size_t rows_above)
{
    reach(frames, draw, ImGuiKey_End);
    for(std::size_t step = 0; step <= rows_above; ++step)
        tap(frames, draw, ImGuiKey_UpArrow);

    walk_to_end(frames, draw, ImGuiKey_LeftArrow);
}

template<typename panel_type>
void press_append(panel_type &panel)
{
    imgui_frame frames;
    const drawing draw = over(panel);
    start_navigating(frames, draw);
    reach(frames, draw, ImGuiKey_End);
    tap(frames, draw, ImGuiKey_Space);
}

// The control that takes a row out stands at the end of it, past the values.
template<typename panel_type>
void press_remove(panel_type &panel, std::size_t rows_above)
{
    imgui_frame frames;
    const drawing draw = over(panel);
    start_navigating(frames, draw);
    reach_row(frames, draw, rows_above);
    walk_to_end(frames, draw, ImGuiKey_RightArrow);
    tap(frames, draw, ImGuiKey_Space);
}

// The first row offers no way up, so the arrow that carries a row down is the only one it draws and
// the one the walk lands on; every other row draws both, with the upward one first.
template<typename panel_type>
void press_raise(panel_type &panel, std::size_t rows_above)
{
    imgui_frame frames;
    const drawing draw = over(panel);
    start_navigating(frames, draw);
    reach_row(frames, draw, rows_above);
    tap(frames, draw, ImGuiKey_Space);
}

template<typename panel_type>
void press_lower(panel_type &panel, std::size_t rows_above)
{
    imgui_frame frames;
    const drawing draw = over(panel);
    start_navigating(frames, draw);
    reach_row(frames, draw, rows_above);
    if(rows_above + 1u != panel.state().rows.size())
        tap(frames, draw, ImGuiKey_RightArrow);
    tap(frames, draw, ImGuiKey_Space);
}

// Every control the row offers the keyboard, walked from the leftmost of them rightwards.
template<typename panel_type>
std::size_t controls_along_row(panel_type &panel, std::size_t rows_above)
{
    imgui_frame frames;
    const drawing draw = over(panel);
    start_navigating(frames, draw);
    reach_row(frames, draw, rows_above);

    std::size_t counted = 0;
    for(ImGuiID standing = 0; standing != standing_on(); ++counted)
    {
        standing = standing_on();
        tap(frames, draw, ImGuiKey_RightArrow);
    }

    return counted;
}

}

TEST_CASE("a list window built over an arm that has published nothing refuses and names both ends", "[manipulator][waypoints]")
{
    arm_publisher unheld;

    REQUIRE_THROWS_MATCHES(joint_waypoint_list("Waypoints", unheld.reader(), frames_of), std::invalid_argument,
                           Message("praxis: the edited list window was given no published arm state to hold"));
    REQUIRE_THROWS_MATCHES(pose_waypoint_list("Waypoints", unheld.reader(), frames_of), std::invalid_argument,
                           Message("praxis: the edited list window was given no published arm state to hold"));
}

TEST_CASE("a list opened with three rows shows three rows and reports them in the order they stood", "[manipulator][waypoints]")
{
    const std::shared_ptr<arm_publisher> published = publishing(two_joint_arm_at_rest());
    joint_waypoint_list configurations("Waypoints", published->reader(), frames_of, joint_waypoint_list::settings{three_configurations()});
    pose_waypoint_list poses("Poses", published->reader(), frames_of, pose_waypoint_list::settings{three_poses()});

    const joint_waypoint_list::settings held = configurations.state();
    const pose_waypoint_list::settings posed = poses.state();

    REQUIRE(held.rows.size() == 3u);
    stands_at(held.rows[0], row_at(10.0, 20.0));
    stands_at(held.rows[2], row_at(50.0, 60.0));
    REQUIRE(posed.rows.size() == 3u);
    stands_at(posed.rows[0], pose_at(0.1f, 10.f));
    stands_at(posed.rows[2], pose_at(0.3f, 30.f));
    CHECK(geometry_of([&configurations] { configurations.render(); }) != stating_absence("Waypoints"));
    CHECK(geometry_of([&poses] { poses.render(); }) != stating_absence("Poses"));
}

TEST_CASE("a list opened with no row at all opens at the rows its own kind ships", "[manipulator][waypoints]")
{
    const std::shared_ptr<arm_publisher> published = publishing(two_joint_arm_at_rest());
    joint_waypoint_list configurations("Waypoints", published->reader(), frames_of);
    pose_waypoint_list poses("Poses", published->reader(), frames_of);

    const std::vector<joint_vector> opened   = joint_waypoint_list::opening_rows(2u);
    const std::vector<edited_pose> opened_at = pose_waypoint_list::opening_rows(2u);
    const joint_waypoint_list::settings held = configurations.state();
    const pose_waypoint_list::settings posed = poses.state();

    REQUIRE(opened.size() >= 3u);
    REQUIRE(held.rows.size() == opened.size());
    for(std::size_t row = 0; row < opened.size(); ++row)
        stands_at(held.rows[row], opened[row]);

    REQUIRE(opened_at.size() >= 3u);
    REQUIRE(posed.rows.size() == opened_at.size());
    for(std::size_t row = 0; row < opened_at.size(); ++row)
        stands_at(posed.rows[row], opened_at[row]);
}

TEST_CASE("appending to a configuration list puts the joints the publication reports at the end of it", "[manipulator][waypoints]")
{
    const std::shared_ptr<arm_publisher> published = publishing(at_rest(configuration(0.25, -0.5), reached, upright));
    joint_waypoint_list panel("Waypoints", published->reader(), frames_of, joint_waypoint_list::settings{three_configurations()});

    press_append(panel);

    const joint_waypoint_list::settings held = panel.state();

    REQUIRE(held.rows.size() == 4u);
    stands_at(held.rows.back(), configuration(0.25, -0.5));
    stands_at(held.rows[2], row_at(50.0, 60.0));
}

TEST_CASE("appending to a pose list puts the tool pose the publication reports at the end of it", "[manipulator][waypoints]")
{
    const std::shared_ptr<arm_publisher> published = publishing(two_joint_arm_at_rest());
    pose_waypoint_list panel("Poses", published->reader(), frames_of, pose_waypoint_list::settings{three_poses()});

    press_append(panel);

    const pose_waypoint_list::settings held = panel.state();

    REQUIRE(held.rows.size() == 4u);
    CHECK(held.rows.back().position.isApprox(reached.cast<float>(), static_cast<float>(float_step)));
    CHECK(held.rows.back().euler_degrees.isZero(static_cast<float>(float_step)));
    stands_at(held.rows[2], pose_at(0.3f, 30.f));
}

TEST_CASE("a pose list over a publication carrying no tool pose is left as it stands and says so", "[manipulator][waypoints]")
{
    const std::shared_ptr<arm_publisher> published = publishing(arm_without_a_tool_pose());
    pose_waypoint_list panel("Poses", published->reader(), frames_of, pose_waypoint_list::settings{three_poses()});

    const std::string said = reported_by([&panel] { press_append(panel); });

    CHECK(panel.state().rows.size() == 3u);
    CHECK_THAT(said, ContainsSubstring("asked for a row from a publication carrying no tool pose"));
}

TEST_CASE("removing a row leaves the rows beside it in the order they stood", "[manipulator][waypoints]")
{
    const std::shared_ptr<arm_publisher> published = publishing(two_joint_arm_at_rest());
    joint_waypoint_list configurations("Waypoints", published->reader(), frames_of, joint_waypoint_list::settings{three_configurations()});
    pose_waypoint_list poses("Poses", published->reader(), frames_of, pose_waypoint_list::settings{three_poses()});

    press_remove(configurations, 1u);
    press_remove(poses, 1u);

    const joint_waypoint_list::settings held = configurations.state();
    const pose_waypoint_list::settings posed = poses.state();

    REQUIRE(held.rows.size() == 2u);
    stands_at(held.rows[0], row_at(10.0, 20.0));
    stands_at(held.rows[1], row_at(50.0, 60.0));
    REQUIRE(posed.rows.size() == 2u);
    stands_at(posed.rows[0], pose_at(0.1f, 10.f));
    stands_at(posed.rows[1], pose_at(0.3f, 30.f));
}

TEST_CASE("raising a row changes the order the list reports and nothing else", "[manipulator][waypoints]")
{
    const std::shared_ptr<arm_publisher> published = publishing(two_joint_arm_at_rest());
    joint_waypoint_list configurations("Waypoints", published->reader(), frames_of, joint_waypoint_list::settings{three_configurations()});
    pose_waypoint_list poses("Poses", published->reader(), frames_of, pose_waypoint_list::settings{three_poses()});

    press_raise(configurations, 0u);
    press_raise(poses, 0u);

    const joint_waypoint_list::settings held = configurations.state();
    const pose_waypoint_list::settings posed = poses.state();

    REQUIRE(held.rows.size() == 3u);
    stands_at(held.rows[0], row_at(10.0, 20.0));
    stands_at(held.rows[1], row_at(50.0, 60.0));
    stands_at(held.rows[2], row_at(30.0, 40.0));
    REQUIRE(posed.rows.size() == 3u);
    stands_at(posed.rows[1], pose_at(0.3f, 30.f));
    stands_at(posed.rows[2], pose_at(0.2f, 20.f));
}

TEST_CASE("lowering a row changes the order the list reports and nothing else", "[manipulator][waypoints]")
{
    const std::shared_ptr<arm_publisher> published = publishing(two_joint_arm_at_rest());
    joint_waypoint_list configurations("Waypoints", published->reader(), frames_of, joint_waypoint_list::settings{three_configurations()});
    pose_waypoint_list poses("Poses", published->reader(), frames_of, pose_waypoint_list::settings{three_poses()});

    press_lower(configurations, 2u);
    press_lower(poses, 2u);

    const joint_waypoint_list::settings held = configurations.state();
    const pose_waypoint_list::settings posed = poses.state();

    REQUIRE(held.rows.size() == 3u);
    stands_at(held.rows[0], row_at(30.0, 40.0));
    stands_at(held.rows[1], row_at(10.0, 20.0));
    stands_at(held.rows[2], row_at(50.0, 60.0));
    REQUIRE(posed.rows.size() == 3u);
    stands_at(posed.rows[0], pose_at(0.2f, 20.f));
    stands_at(posed.rows[1], pose_at(0.1f, 10.f));
    stands_at(posed.rows[2], pose_at(0.3f, 30.f));
}

// A row at either end cannot be carried past it, so the control that would is not one the keyboard
// reaches: each end offers one control fewer than a row between them, and the one it keeps is the
// arrow pointing back into the list.
TEST_CASE("neither end of the list offers the control that would carry a row past it", "[manipulator][waypoints]")
{
    const std::shared_ptr<arm_publisher> published = publishing(two_joint_arm_at_rest());
    joint_waypoint_list configurations("Waypoints", published->reader(), frames_of, joint_waypoint_list::settings{three_configurations()});
    pose_waypoint_list poses("Poses", published->reader(), frames_of, pose_waypoint_list::settings{three_poses()});

    const std::size_t between = controls_along_row(configurations, 1u);

    REQUIRE(between > 2u);
    CHECK(controls_along_row(configurations, 2u) + 1u == between);
    CHECK(controls_along_row(configurations, 0u) + 1u == between);

    const std::size_t posed_between = controls_along_row(poses, 1u);

    REQUIRE(posed_between > 2u);
    CHECK(controls_along_row(poses, 2u) + 1u == posed_between);
    CHECK(controls_along_row(poses, 0u) + 1u == posed_between);
}

// What the first row keeps is read by pressing it: the arrow it leads with carries it down, which is
// the one a row that can go no higher is left with.
TEST_CASE("the control the first row leads with carries it down the list", "[manipulator][waypoints]")
{
    const std::shared_ptr<arm_publisher> published = publishing(two_joint_arm_at_rest());
    joint_waypoint_list configurations("Waypoints", published->reader(), frames_of, joint_waypoint_list::settings{three_configurations()});

    press_raise(configurations, 2u);

    const joint_waypoint_list::settings held = configurations.state();

    REQUIRE(held.rows.size() == 3u);
    stands_at(held.rows[0], row_at(30.0, 40.0));
    stands_at(held.rows[1], row_at(10.0, 20.0));
    stands_at(held.rows[2], row_at(50.0, 60.0));
}

// Whichever kind a row is, its columns are named by the row rather than by the list holding it: an
// arm's joints for a configuration, the position and the angles for a pose.
TEST_CASE("each kind of row names the columns its own values stand under", "[manipulator][waypoints]")
{
    CHECK(list_row_traits<joint_vector>::column_labels(6u) == std::vector<std::string>{"j1", "j2", "j3", "j4", "j5", "j6"});
    CHECK(list_row_traits<edited_pose>::column_labels(6u) == std::vector<std::string>{"X", "Y", "Z", "A", "B", "C"});
}

// The rows are typed in degrees and answered in radians, so a list opened at a value and read back
// at it went through the conversion in both directions rather than through neither.
TEST_CASE("a configuration list answers its rows in the unit every other surface carries", "[manipulator][waypoints]")
{
    const std::shared_ptr<arm_publisher> published = publishing(two_joint_arm_at_rest());
    joint_waypoint_list panel("Waypoints", published->reader(), frames_of, joint_waypoint_list::settings{{row_at(90.0, -180.0)}});

    const joint_waypoint_list::settings held = panel.state();

    REQUIRE(held.rows.size() == 1u);
    CHECK(std::abs(held.rows.front()[0] - std::numbers::pi / 2.0) < float_step);
    CHECK(std::abs(held.rows.front()[1] + std::numbers::pi) < float_step);
}

TEST_CASE("a configuration row whose width is not the joint count is declined by name and the rows beside it stand", "[manipulator][waypoints]")
{
    const std::shared_ptr<arm_publisher> published = publishing(two_joint_arm_at_rest());
    joint_vector wide(3);
    wide << 0.1, 0.2, 0.3;

    std::size_t held       = 0;
    const std::string said = reported_by(
            [&published, &wide, &held]
            {
                joint_waypoint_list panel("Waypoints", published->reader(), frames_of, joint_waypoint_list::settings{{row_at(10.0, 20.0), wide, row_at(50.0, 60.0)}});
                held = panel.state().rows.size();
            });

    CHECK(held == 2u);
    CHECK_THAT(said, ContainsSubstring("manipulator.edited_list_window"));
    CHECK_THAT(said, ContainsSubstring("a row of 3 joint values for an arm of 2 joints at row 2"));
    CHECK_THAT(said, ContainsSubstring("the rows beside it still stand"));
}

TEST_CASE("a configuration list over a publication reporting no joints shows no row and declines an append", "[manipulator][waypoints]")
{
    const std::shared_ptr<arm_publisher> published = publishing(jointless_arm_at_rest());
    joint_waypoint_list panel("Waypoints", published->reader(), frames_of);

    REQUIRE(panel.state().rows.empty());

    const std::string said = reported_by([&panel] { press_append(panel); });

    CHECK(panel.state().rows.empty());
    CHECK_THAT(said, ContainsSubstring("asked for a row on an arm of no joints"));
}

TEST_CASE("a list no key path was named for offers nothing", "[manipulator][waypoints]")
{
    const std::shared_ptr<arm_publisher> published = publishing(two_joint_arm_at_rest());
    joint_waypoint_list unnamed("Waypoints", published->reader(), frames_of, joint_waypoint_list::settings{three_configurations()});
    joint_waypoint_list named("Waypoints", published->reader(), frames_of, joint_waypoint_list::settings{three_configurations()}, "machine/joint_waypoints");
    pose_waypoint_list unnamed_poses("Poses", published->reader(), frames_of, pose_waypoint_list::settings{three_poses()});
    pose_waypoint_list named_poses("Poses", published->reader(), frames_of, pose_waypoint_list::settings{three_poses()}, "machine/pose_waypoints");

    CHECK(unnamed.as_configurable() == nullptr);
    CHECK(named.as_configurable() == &named);
    CHECK(named.settings_path() == "machine/joint_waypoints");
    CHECK(unnamed_poses.as_configurable() == nullptr);
    CHECK(named_poses.as_configurable() == &named_poses);
    CHECK(named_poses.settings_path() == "machine/pose_waypoints");
}
