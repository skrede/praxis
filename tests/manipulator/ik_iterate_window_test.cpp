#include "window_stage.h"

#include "praxis/manipulator/ik_iterate_window.h"

#include "praxis/scheduler/scheduler.h"

#include "praxis/rigid_motion/capabilities.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_exception.hpp>

#include <imgui.h>

#include <Eigen/Core>

#include <span>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <stdexcept>

using namespace praxis;
using namespace praxis::tests;
using namespace praxis::fixture;
using namespace praxis::scheduler;
using namespace praxis::manipulator;
using Catch::Matchers::Message;

namespace {

constexpr const char *panel_title = "Iterates";

const Eigen::Vector3d origin(Eigen::Vector3d::Zero());
const rotation upright(rotation::Identity());

// Every configuration and every error below is a value binary carries exactly, so a step compared
// against the one it was recorded as cannot agree by rounding.
std::vector<iteration_state> stepping(double posture, std::uint32_t steps)
{
    std::vector<iteration_state> taken;
    double error = 1.0;
    for(std::uint32_t step = 0; step < steps; ++step)
    {
        taken.push_back(iteration_state{configuration(posture, 0.125 * static_cast<double>(step)), error, 0.5 * error, error, step});
        error *= 0.5;
    }

    return taken;
}

arm_snapshot standing_with(const joint_vector &joints, const std::vector<std::vector<iteration_state>> &sequences)
{
    arm_snapshot seen = at_rest(joints, origin, upright);
    seen.iterations   = sequences;

    return seen;
}

bool same_step(const iteration_state &held, const iteration_state &taken)
{
    return held.index == taken.index && held.angular_error == taken.angular_error && held.linear_error == taken.linear_error && held.step_norm == taken.step_norm &&
            held.joint_positions.size() == taken.joint_positions.size() && (held.joint_positions.array() == taken.joint_positions.array()).all();
}

bool same_sequences(const std::vector<std::vector<iteration_state>> &held, const std::vector<std::vector<iteration_state>> &taken)
{
    if(held.size() != taken.size())
        return false;

    for(std::size_t which = 0; which < held.size(); ++which)
    {
        if(held[which].size() != taken[which].size())
            return false;
        for(std::size_t step = 0; step < held[which].size(); ++step)
            if(!same_step(held[which][step], taken[which][step]))
                return false;
    }

    return true;
}

// What the joint-space route was handed, one entry per motion asked for. The factory refuses, so a
// case reads the waypoints without a motion having to play out and the arm stays idle.
std::vector<std::vector<joint_vector>> handed;

expected<std::unique_ptr<trajectory::trajectory_generator>, refusal> recorded_waypoints(std::span<const trajectory::configuration> waypoints, const trajectory::configuration &,
                                                                                        const trajectory::configuration_limits &)
{
    handed.emplace_back(waypoints.begin(), waypoints.end());

    return praxis::unexpected(refusal::no_solution);
}

// The steps one solve reports: as many as the target's second component names, both errors halving
// each step, and the posture the first component names.
expected<void, refusal> stepping_inverse_kinematics(const forward_kinematics_ops &, const differential_kinematics_ops &, const screw_chain &, const transform &desired,
                                                    const joint_vector &, const solver_parameters &, ik_result &answer)
{
    answer.iterations = stepping(desired(0, 3), static_cast<std::uint32_t>(std::lround(desired(1, 3))));
    answer.solutions.push_back(configuration(desired(0, 3), 0.0));

    return {};
}

expected<joint_vector, refusal> solving_task_space_pose(const kinematics &solver, const transform &pose, const joint_vector &j0)
{
    return solver.ik_solve(pose, j0, solver_parameters{});
}

transform target_of(double posture, double steps)
{
    transform pose = transform::Identity();
    pose(0, 3)     = posture;
    pose(1, 3)     = steps;

    return pose;
}

composed_arm solving_arm(praxis::scheduler::scheduler &loop)
{
    const strand work = *loop.make_strand();
    const auto driven = std::make_shared<scene_robot>(
            scene_robot::compose(kinematics::compose(sliding_chain(), forward_kinematics_ops{.forward_kinematics = &sliding_forward_kinematics}, differential_kinematics_ops{},
                                                     inverse_kinematics_ops{.inverse_kinematics = &stepping_inverse_kinematics}, rigid_motion::baseline().screw,
                                                     rigid_motion::baseline().frame)
                                         .value(),
                                 robot_ops{}, rigid_motion::baseline().frame, 2u)
                    .value());
    const auto published = std::make_shared<arm_publisher>();
    const auto control =
            std::make_shared<robot_controller>(*driven, motion_ops{.task_space_pose = &solving_task_space_pose}, trajectory::path_ops{}, task_trajectory_ops{},
                                               trajectory::time_scaling_ops{}, trajectory::trajectory_ops{.joint_space_waypoints = &recorded_waypoints}, rigid_motion::screw_ops{});

    return composed_arm{published->reader(), published, std::make_shared<owned_arm>(work, work, driven, control, published)};
}

// Three solves inside one work item, which is one command extent and therefore one set of recorded
// sequences, as a solve over three starts leaves.
void solve_three(const composed_arm &arm, praxis::scheduler::scheduler &loop)
{
    command(arm.owned,
            [](robot_controller &control, scene_robot &)
            {
                control.preview_task_space_pose(target_of(0.25, 3));
                control.preview_task_space_pose(target_of(0.5, 4));
                control.preview_task_space_pose(target_of(0.75, 5));
            });
    static_cast<void>(loop.drain());
}

std::shared_ptr<arm_publisher> carrying(const std::vector<std::vector<iteration_state>> &sequences)
{
    return publishing(standing_with(configuration(0.0, 0.0), sequences));
}

void press_last(imgui_frame &frames, const drawing &draw)
{
    reach(frames, draw, ImGuiKey_End);
    tap(frames, draw, ImGuiKey_Space);
}

}

TEST_CASE("an iterate table built over an arm that has published nothing refuses and names the end", "[manipulator][iterates]")
{
    arm_publisher unheld;

    REQUIRE_THROWS_MATCHES(ik_iterate_window(panel_title, unheld.reader(), std::weak_ptr<owned_arm>()), std::invalid_argument,
                           Message("praxis: the iterate table window was given no published arm state to hold"));
}

TEST_CASE("a publication carrying three sequences is about the start the settings name and carries that start's steps", "[manipulator][iterates]")
{
    const std::shared_ptr<arm_publisher> published = carrying({stepping(0.25, 3), stepping(0.5, 4), stepping(0.75, 5)});
    ik_iterate_window panel(panel_title, published->reader(), std::weak_ptr<owned_arm>(), ik_iterate_window::settings{2u});

    REQUIRE(panel.selected().has_value());
    CHECK(*panel.selected() == 2u);
    REQUIRE(panel.iterates().size() == 5u);
    CHECK(same_step(panel.iterates().front(), stepping(0.75, 5).front()));
}

TEST_CASE("which start the table is about is the window's own and does not follow the arm", "[manipulator][iterates]")
{
    const std::vector<std::vector<iteration_state>> three{stepping(0.25, 3), stepping(0.5, 4), stepping(0.75, 5)};
    const std::shared_ptr<arm_publisher> published = carrying(three);
    ik_iterate_window panel(panel_title, published->reader(), std::weak_ptr<owned_arm>(), ik_iterate_window::settings{2u});

    imgui_frame frames;
    frames.draw(over(panel));

    // The arm is driven somewhere else and two of the starts are made to have converged to one
    // posture: neither is a reason for the table to be about a different start.
    published->publish(std::make_shared<const arm_snapshot>(standing_with(configuration(0.75, 0.0), {three[0], three[2], three[2]})));
    frames.draw(over(panel));

    REQUIRE(panel.selected().has_value());
    CHECK(*panel.selected() == 2u);
    CHECK(panel.state().start == 2u);
}

TEST_CASE("a start that recorded no step is still a start, and carries no step rather than being absent", "[manipulator][iterates]")
{
    const std::shared_ptr<arm_publisher> published = carrying({stepping(0.25, 3), {}, stepping(0.75, 5)});
    ik_iterate_window panel(panel_title, published->reader(), std::weak_ptr<owned_arm>(), ik_iterate_window::settings{1u});

    imgui_frame frames;
    frames.assert_on_frame_faults(true);
    frames.draw(over(panel));

    REQUIRE(panel.selected().has_value());
    CHECK(*panel.selected() == 1u);
    CHECK(panel.iterates().empty());
    CHECK(frames.has_draw_data());
}

TEST_CASE("a publication carrying no sequence at all is about no start and hands the arm nothing", "[manipulator][iterates]")
{
    praxis::scheduler::scheduler loop(inline_workers, clock_source{&reading});
    const composed_arm arm = solving_arm(loop);
    handed.clear();
    ik_iterate_window panel(panel_title, arm.seen, arm.owned);

    imgui_frame frames;
    const drawing draw = over(panel);
    start_navigating(frames, draw);
    press_last(frames, draw);
    static_cast<void>(loop.drain());

    CHECK_FALSE(panel.selected().has_value());
    CHECK(panel.iterates().empty());
    CHECK(handed.empty());
}

// The row is the whole request: one configuration, which is the step itself rather than a sweep
// between steps, and the joint-space route is asked for nothing beside it.
TEST_CASE("stepping to a row hands the joint-space route that step alone", "[manipulator][iterates]")
{
    praxis::scheduler::scheduler loop(inline_workers, clock_source{&reading});
    const composed_arm arm = solving_arm(loop);
    solve_three(arm, loop);
    handed.clear();

    ik_iterate_window panel(panel_title, arm.seen, arm.owned, ik_iterate_window::settings{1u, control_mode::simulation});
    imgui_frame frames;
    const drawing draw = over(panel);
    start_navigating(frames, draw);
    press_last(frames, draw);
    static_cast<void>(loop.drain());

    const joint_vector standing = stepping(0.5, 4).back().joint_positions;

    REQUIRE(handed.size() == 1u);
    REQUIRE(handed.front().size() == 1u);
    CHECK((handed.front().front().array() == standing.array()).all());
}

TEST_CASE("the steps a solve recorded stand unchanged through being read", "[manipulator][iterates]")
{
    praxis::scheduler::scheduler loop(inline_workers, clock_source{&reading});
    const composed_arm arm = solving_arm(loop);
    solve_three(arm, loop);
    handed.clear();

    const std::vector<std::vector<iteration_state>> before = arm.seen.read()->iterations;
    REQUIRE(before.size() == 3u);

    ik_iterate_window panel(panel_title, arm.seen, arm.owned, ik_iterate_window::settings{1u, control_mode::simulation});
    imgui_frame frames;
    const drawing draw = over(panel);
    start_navigating(frames, draw);
    press_last(frames, draw);
    static_cast<void>(loop.drain());

    REQUIRE(handed.size() == 1u);
    CHECK(same_sequences(before, arm.seen.read()->iterations));
}

TEST_CASE("stepping to a row puts the arm at that step's configuration and not at an interpolation of it", "[manipulator][iterates]")
{
    praxis::scheduler::scheduler loop(inline_workers, clock_source{&reading});
    const composed_arm arm = solving_arm(loop);
    solve_three(arm, loop);
    handed.clear();

    ik_iterate_window panel(panel_title, arm.seen, arm.owned, ik_iterate_window::settings{1u, control_mode::preview});
    imgui_frame frames;
    const drawing draw = over(panel);
    start_navigating(frames, draw);
    press_last(frames, draw);
    static_cast<void>(loop.drain());

    const joint_vector standing                     = stepping(0.5, 4).back().joint_positions;
    const std::shared_ptr<const arm_snapshot> after = arm.seen.read();

    REQUIRE(after != nullptr);
    REQUIRE(after->joints.size() == standing.size());
    CHECK((after->joints.array() == standing.array()).all());
    CHECK(handed.empty());
}
