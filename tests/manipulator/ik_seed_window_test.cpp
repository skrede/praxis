#include "window_stage.h"
#include "captured_log.h"

#include "praxis/manipulator/ik_seed_window.h"

#include "praxis/rigid_motion/angles.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>
#include <catch2/matchers/catch_matchers_exception.hpp>

#include <Eigen/Core>

#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <stdexcept>

using namespace praxis;
using namespace praxis::tests;
using namespace praxis::fixture;
using namespace praxis::manipulator;
using Catch::Matchers::Message;
using Catch::Matchers::ContainsSubstring;

namespace {

// Both ends of a row hold it in single precision, so a value that made the trip is the value behind
// it to within one float step of it, in radians.
constexpr double float_step = 1.0e-5;

const Eigen::Vector3d origin(Eigen::Vector3d::Zero());
const rotation upright(rotation::Identity());

arm_snapshot two_joint_arm_at_rest()
{
    return at_rest(configuration(0.0, 0.0), origin, upright);
}

arm_snapshot jointless_arm_at_rest()
{
    return at_rest(joint_vector(), origin, upright);
}

joint_vector start_at(double first, double second)
{
    return configuration(first * radians_per_degree, second * radians_per_degree);
}

std::vector<joint_vector> three_starts()
{
    return {start_at(10.0, 20.0), start_at(30.0, 40.0), start_at(50.0, 60.0)};
}

void stands_at(const joint_vector &read, const joint_vector &expected)
{
    REQUIRE(read.size() == expected.size());
    CHECK(read.isApprox(expected, float_step));
}

// The list's own controls stand at the left of every row, in a column with the control that appends
// one, so the row a step upward reaches is the row above whatever the values beside it are.
void press_append(imgui_frame &frames, const drawing &draw)
{
    reach(frames, draw, ImGuiKey_End);
    tap(frames, draw, ImGuiKey_Space);
}

void press_remove(imgui_frame &frames, const drawing &draw, std::size_t rows_above)
{
    reach(frames, draw, ImGuiKey_End);
    for(std::size_t step = 0; step <= rows_above; ++step)
        tap(frames, draw, ImGuiKey_UpArrow);
    tap(frames, draw, ImGuiKey_Space);
}

void press_raise(imgui_frame &frames, const drawing &draw, std::size_t rows_above)
{
    reach(frames, draw, ImGuiKey_End);
    for(std::size_t step = 0; step <= rows_above; ++step)
        tap(frames, draw, ImGuiKey_UpArrow);
    tap(frames, draw, ImGuiKey_RightArrow);
    tap(frames, draw, ImGuiKey_Space);
}

}

TEST_CASE("a seed list window built over an arm that has published nothing refuses and names both ends", "[manipulator][seeds]")
{
    arm_publisher unheld;

    REQUIRE_THROWS_MATCHES(ik_seed_window("Starts", unheld.reader()), std::invalid_argument, Message("praxis: the seed list window was given no published arm state to hold"));
}

TEST_CASE("a seed list window opened with three starts shows three rows and reports three starts", "[manipulator][seeds]")
{
    const std::shared_ptr<arm_publisher> published = publishing(two_joint_arm_at_rest());
    ik_seed_window panel("Starts", published->reader(), ik_seed_window::settings{three_starts()});

    const ik_seed_window::settings held = panel.state();

    REQUIRE(held.seeds.size() == 3u);
    stands_at(held.seeds[0], start_at(10.0, 20.0));
    stands_at(held.seeds[1], start_at(30.0, 40.0));
    stands_at(held.seeds[2], start_at(50.0, 60.0));
    CHECK(geometry_of([&panel] { panel.render(); }) != stating_absence("Starts"));
}

TEST_CASE("a seed list window opened with no start at all opens at the spread the joint count names", "[manipulator][seeds]")
{
    const std::shared_ptr<arm_publisher> published = publishing(two_joint_arm_at_rest());
    ik_seed_window panel("Starts", published->reader());

    const std::vector<joint_vector> opened = ik_seed_window::opening_seeds(2u);
    const ik_seed_window::settings held    = panel.state();

    REQUIRE(opened.size() == 8u);
    REQUIRE(held.seeds.size() == opened.size());
    for(std::size_t start = 0; start < opened.size(); ++start)
        stands_at(held.seeds[start], opened[start]);
    CHECK(opened.front().isZero(float_step));
}

TEST_CASE("appending a start puts a row at the joint count the publication reports", "[manipulator][seeds]")
{
    const std::shared_ptr<arm_publisher> published = publishing(two_joint_arm_at_rest());
    ik_seed_window panel("Starts", published->reader(), ik_seed_window::settings{three_starts()});

    imgui_frame frames;
    const drawing draw = over(panel);
    start_navigating(frames, draw);
    press_append(frames, draw);

    const ik_seed_window::settings held = panel.state();

    REQUIRE(held.seeds.size() == 4u);
    CHECK(held.seeds.back().size() == 2);
    CHECK(held.seeds.back().isZero(float_step));
    stands_at(held.seeds[2], start_at(50.0, 60.0));
}

TEST_CASE("removing a start leaves the starts beside it in the order they stood", "[manipulator][seeds]")
{
    const std::shared_ptr<arm_publisher> published = publishing(two_joint_arm_at_rest());
    ik_seed_window panel("Starts", published->reader(), ik_seed_window::settings{three_starts()});

    imgui_frame frames;
    const drawing draw = over(panel);
    start_navigating(frames, draw);
    press_remove(frames, draw, 1u);

    const ik_seed_window::settings held = panel.state();

    REQUIRE(held.seeds.size() == 2u);
    stands_at(held.seeds[0], start_at(10.0, 20.0));
    stands_at(held.seeds[1], start_at(50.0, 60.0));
}

TEST_CASE("raising a start changes the order the list reports and nothing else", "[manipulator][seeds]")
{
    const std::shared_ptr<arm_publisher> published = publishing(two_joint_arm_at_rest());
    ik_seed_window panel("Starts", published->reader(), ik_seed_window::settings{three_starts()});

    imgui_frame frames;
    const drawing draw = over(panel);
    start_navigating(frames, draw);
    press_raise(frames, draw, 0u);

    const ik_seed_window::settings held = panel.state();

    REQUIRE(held.seeds.size() == 3u);
    stands_at(held.seeds[0], start_at(10.0, 20.0));
    stands_at(held.seeds[1], start_at(50.0, 60.0));
    stands_at(held.seeds[2], start_at(30.0, 40.0));
}

TEST_CASE("a start whose width is not the joint count is declined by name and the list is as it was", "[manipulator][seeds]")
{
    const std::shared_ptr<arm_publisher> published = publishing(two_joint_arm_at_rest());
    joint_vector wide(3);
    wide << 0.1, 0.2, 0.3;

    std::size_t held       = 0;
    const std::string said = reported_by(
            [&published, &wide, &held]
            {
                ik_seed_window panel("Starts", published->reader(), ik_seed_window::settings{{start_at(10.0, 20.0), wide, start_at(50.0, 60.0)}});
                held = panel.state().seeds.size();
            });

    CHECK(held == 2u);
    CHECK_THAT(said, ContainsSubstring("manipulator.ik_seed_window"));
    CHECK_THAT(said, ContainsSubstring("start of 3 joint values at row 2 for an arm of 2 joints"));
}

TEST_CASE("a seed list window over a publication reporting no joints shows no row and declines an append", "[manipulator][seeds]")
{
    const std::shared_ptr<arm_publisher> published = publishing(jointless_arm_at_rest());
    ik_seed_window panel("Starts", published->reader());

    REQUIRE(panel.state().seeds.empty());

    const std::string said = reported_by(
            [&panel]
            {
                imgui_frame frames;
                const drawing draw = over(panel);
                start_navigating(frames, draw);
                press_append(frames, draw);
            });

    CHECK(panel.state().seeds.empty());
    CHECK_THAT(said, ContainsSubstring("asked for a start on an arm of no joints"));
}

TEST_CASE("a seed list window no key path was named for offers nothing", "[manipulator][seeds]")
{
    const std::shared_ptr<arm_publisher> published = publishing(two_joint_arm_at_rest());
    ik_seed_window unnamed("Starts", published->reader(), ik_seed_window::settings{three_starts()});
    ik_seed_window named("Starts", published->reader(), ik_seed_window::settings{three_starts()}, "machine/ik_seeds");

    CHECK(unnamed.as_configurable() == nullptr);
    CHECK(named.as_configurable() == &named);
    CHECK(named.settings_path() == "machine/ik_seeds");
}
