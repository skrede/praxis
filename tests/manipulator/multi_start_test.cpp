#include "fixtures.h"

#include "captured_log.h"

#include "praxis/manipulator/robot_controller.h"

#include "praxis/evaluation/tolerance.h"

#include "praxis/scheduler/scheduler.h"

#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_string.hpp>

#include <Eigen/Core>

#include <span>
#include <cmath>
#include <chrono>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <numbers>

using namespace praxis;
using namespace praxis::tests;
using namespace praxis::fixture;
using namespace praxis::scheduler;
using namespace praxis::manipulator;

namespace {

constexpr double reached_at = 1.0e-6;
constexpr step_period stepped{seconds{0.01}};
constexpr std::uint32_t most_steps = 20000;

time_point dictated{};

time_point reading()
{
    return dictated;
}

clock_source dictating()
{
    dictated = time_point{};

    return clock_source{&reading};
}

// Wide enough that a posture a full turn from where the arm stands is a motion the bounds shape
// rather than one they forbid.
screw_chain reaching_chain()
{
    joint_limits bounds{};
    bounds.velocity       = joint_vector::Constant(2, 20.0);
    bounds.acceleration   = joint_vector::Constant(2, 80.0);
    bounds.lower_position = joint_vector::Constant(2, -10.0);
    bounds.upper_position = joint_vector::Constant(2, 10.0);

    return screw_chain(transform::Identity(), {screw_axis::Zero(), screw_axis::Zero()}, bounds);
}

// A stub whose answer each case dictates through the seed it hands it: the first value names the
// posture found, the magnitude of the second names how many iterates the search reported, and a
// negative second value is a start that entered the solve and converged on nothing.
expected<void, refusal> the_posture_the_seed_names(const forward_kinematics_ops &, const differential_kinematics_ops &, const screw_chain &, const transform &, const joint_vector &seed,
                                                   const solver_parameters &, ik_result &answer)
{
    const std::uint32_t iterates = static_cast<std::uint32_t>(std::lround(std::abs(seed[1])));
    for(std::uint32_t taken = 0; taken < iterates; ++taken)
        answer.iterations.push_back(iteration_state{configuration(seed[0], -0.25), 0.0, 0.0, 0.0, taken});

    if(seed[1] > 0.0)
        answer.solutions.push_back(configuration(seed[0], -0.25));

    return {};
}

// Three postures answered in one go and no iterates, which is the shape an answer taken in closed
// form leaves.
expected<void, refusal> three_postures_at_once(const forward_kinematics_ops &, const screw_chain &, const transform &, ik_result &answer)
{
    answer.solutions.push_back(configuration(1.0, 1.0));
    answer.solutions.push_back(configuration(0.5, -0.25));
    answer.solutions.push_back(configuration(-1.0, 1.25));

    return {};
}

inverse_kinematics_ops searching()
{
    return inverse_kinematics_ops{.inverse_kinematics = &the_posture_the_seed_names};
}

inverse_kinematics_ops in_closed_form()
{
    return inverse_kinematics_ops{.analytic_inverse_kinematics = &three_postures_at_once};
}

// The arm, the controller that commands it and the one command extent an arm state would open
// around a work item, so a command over several seeds is one command here as it is there.
struct standing_arm
{
    explicit standing_arm(const inverse_kinematics_ops &inverse)
            : driven(scene_robot::compose(kinematics::compose(reaching_chain(), forward_kinematics_ops{.forward_kinematics = &sliding_forward_kinematics}, differential_kinematics_ops{},
                                                              inverse, rigid_motion::baseline().screw, rigid_motion::baseline().frame)
                                                  .value(),
                                          robot_ops{}, rigid_motion::baseline().frame, 2u)
                             .value())
            , control(driven, motion_ops{}, composing_path(), task_trajectory_ops{}, composing_time_scaling(), trajectory::trajectory_ops{}, rigid_motion::screw_ops{})
    {
        driven.set_joint_positions(configuration(0.25, -0.5));
        control.set_velocity_factor(1.0);
    }

    scene_robot driven;
    robot_controller control;
};

transform target_at(double x)
{
    transform pose = transform::Identity();
    pose(0, 3)     = x;

    return pose;
}

std::vector<joint_vector> seeds_of(std::initializer_list<std::pair<double, double>> named)
{
    std::vector<joint_vector> seeds;
    for(const std::pair<double, double> &one : named)
        seeds.push_back(configuration(one.first, one.second));

    return seeds;
}

// The bound is a failure report rather than a synchronization device.
bool played_out(robot_controller &control)
{
    praxis::scheduler::scheduler loop(inline_workers, dictating());
    const strand work        = *loop.make_strand();
    const task_handle motion = work.every(stepped, overrun::catch_up, [&control](step_delta step) { static_cast<void>(control.advance_playback(step)); });

    for(std::uint32_t taken = 0; taken < most_steps && control.executing(); ++taken)
    {
        dictated += std::chrono::duration_cast<time_point::duration>(stepped.value);
        if(!loop.drain().has_value())
            return false;
    }

    return !control.executing();
}

}

TEST_CASE("starts that converge to one posture leave one distinct solution and one index each naming it", "[manipulator][multi-start]")
{
    standing_arm arm(searching());

    arm.control.solve_from_seeds(target_at(0.5), seeds_of({{0.5, 1.0}, {0.5, 2.0}, {0.5, 3.0}}));

    REQUIRE(arm.control.solutions().size() == 1u);
    CHECK(is_approx_equal(arm.control.solutions()[0], configuration(0.5, -0.25), reached_at));
    REQUIRE(arm.control.reached().size() == 3u);
    CHECK(arm.control.reached()[0] == 0u);
    CHECK(arm.control.reached()[1] == 0u);
    CHECK(arm.control.reached()[2] == 0u);
}

TEST_CASE("starts that reach three postures leave three distinct solutions and one index each naming a different one", "[manipulator][multi-start]")
{
    standing_arm arm(searching());

    arm.control.solve_from_seeds(target_at(0.5), seeds_of({{0.5, 1.0}, {1.0, 1.0}, {-1.0, 1.0}}));

    REQUIRE(arm.control.solutions().size() == 3u);
    CHECK(is_approx_equal(arm.control.solutions()[0], configuration(0.5, -0.25), reached_at));
    CHECK(is_approx_equal(arm.control.solutions()[2], configuration(-1.0, -0.25), reached_at));
    REQUIRE(arm.control.reached().size() == 3u);
    CHECK(arm.control.reached()[0] == 0u);
    CHECK(arm.control.reached()[1] == 1u);
    CHECK(arm.control.reached()[2] == 2u);
}

TEST_CASE("the sequences one command leaves number one per seed, in seed order, whatever the fold did", "[manipulator][multi-start]")
{
    standing_arm arm(searching());

    arm.control.solve_from_seeds(target_at(0.5), seeds_of({{0.5, 1.0}, {0.5, 2.0}, {0.5, 3.0}}));

    REQUIRE(arm.control.solves().size() == 3u);
    CHECK(arm.control.solves()[0].size() == 1u);
    CHECK(arm.control.solves()[1].size() == 2u);
    CHECK(arm.control.solves()[2].size() == 3u);
    CHECK(arm.control.solutions().size() == 1u);
}

TEST_CASE("two postures nearer than the fold are one and two farther apart are two", "[manipulator][multi-start]")
{
    standing_arm arm(searching());

    // A thousandth of a radian apart and a hundredth apart, which stand either side of the distance
    // the fold reads as one posture.
    arm.control.solve_from_seeds(target_at(0.5), seeds_of({{0.5, 1.0}, {0.501, 1.0}, {0.51, 1.0}}));

    REQUIRE(arm.control.solutions().size() == 2u);
    REQUIRE(arm.control.reached().size() == 3u);
    CHECK(arm.control.reached()[0] == 0u);
    CHECK(arm.control.reached()[1] == 0u);
    CHECK(arm.control.reached()[2] == 1u);
}

TEST_CASE("a seed of the wrong width is declined by name and the seeds beside it still run", "[manipulator][multi-start]")
{
    const captured_log recorded;
    standing_arm arm(searching());

    std::vector<joint_vector> seeds = seeds_of({{0.5, 1.0}, {1.0, 1.0}});
    seeds.insert(seeds.begin() + 1, joint_vector::Constant(3, 0.5));

    arm.control.solve_from_seeds(target_at(0.5), seeds);

    CHECK_THAT(recorded.text(), Catch::Matchers::ContainsSubstring("robot_controller.solve_from_seeds"));
    CHECK(arm.control.solutions().size() == 2u);
    CHECK(arm.control.solves().size() == 2u);
    REQUIRE(arm.control.reached().size() == 2u);
    CHECK(arm.control.reached()[0] == 0u);
    CHECK(arm.control.reached()[1] == 1u);
}

TEST_CASE("a start that entered the solve and reached nothing is named past the distinct solutions", "[manipulator][multi-start]")
{
    const captured_log recorded;
    standing_arm arm(searching());

    arm.control.solve_from_seeds(target_at(0.5), seeds_of({{0.5, 1.0}, {0.7, -1.0}, {1.0, 1.0}}));

    CHECK_THAT(recorded.text(), Catch::Matchers::ContainsSubstring("ik.inverse_kinematics"));

    REQUIRE(arm.control.solutions().size() == 2u);
    REQUIRE(arm.control.solves().size() == 3u);
    REQUIRE(arm.control.reached().size() == 3u);
    CHECK(arm.control.reached()[0] == 0u);
    CHECK(arm.control.reached()[1] == arm.control.solutions().size());
    CHECK(arm.control.reached()[2] == 1u);
}

TEST_CASE("a command over no seeds at all leaves the publication where it was and says why", "[manipulator][multi-start]")
{
    const captured_log recorded;
    standing_arm arm(searching());

    arm.control.solve_from_seeds(target_at(0.5), seeds_of({{0.5, 1.0}}));
    REQUIRE(arm.control.solutions().size() == 1u);
    REQUIRE(played_out(arm.control));

    arm.control.solve_from_seeds(target_at(0.9), std::span<const joint_vector>{});

    CHECK_THAT(recorded.text(), Catch::Matchers::ContainsSubstring("no seed"));
    CHECK(arm.control.solutions().size() == 1u);
    CHECK(arm.control.solves().size() == 1u);
    CHECK(arm.control.reached().size() == 1u);
}

TEST_CASE("a command whose every seed is declined starts no solve and leaves nothing standing", "[manipulator][multi-start]")
{
    const captured_log recorded;
    standing_arm arm(searching());

    arm.control.solve_from_seeds(target_at(0.5), seeds_of({{0.9, 1.0}}));
    REQUIRE(arm.control.solutions().size() == 1u);
    REQUIRE(played_out(arm.control));

    const std::vector<joint_vector> all_the_wrong_width{joint_vector::Constant(3, 0.5), joint_vector::Constant(1, 0.5)};
    arm.control.solve_from_seeds(target_at(0.9), all_the_wrong_width);

    CHECK_THAT(recorded.text(), Catch::Matchers::ContainsSubstring("robot_controller.solve_from_seeds"));
    CHECK(arm.control.solves().empty());
    CHECK(arm.control.solutions().empty());
    CHECK(arm.control.reached().empty());
    CHECK_FALSE(arm.control.executing());
}

TEST_CASE("an answer taken in one go publishes the postures it found and one empty sequence", "[manipulator][multi-start]")
{
    standing_arm arm(in_closed_form());

    arm.control.solve_in_closed_form(target_at(0.5));

    REQUIRE(arm.control.solutions().size() == 3u);
    REQUIRE(arm.control.solves().size() == 1u);
    CHECK(arm.control.solves()[0].empty());
    REQUIRE(arm.control.reached().size() == 1u);
    CHECK(arm.control.reached()[0] == 1u);
}

TEST_CASE("an answer taken in one go that is refused publishes its sequence and an index naming no posture", "[manipulator][multi-start]")
{
    const captured_log recorded;
    standing_arm arm(searching());

    arm.control.solve_in_closed_form(target_at(0.5));

    CHECK_THAT(recorded.text(), Catch::Matchers::ContainsSubstring("ik.analytic_inverse_kinematics"));
    CHECK(arm.control.solutions().empty());
    REQUIRE(arm.control.solves().size() == 1u);
    REQUIRE(arm.control.reached().size() == 1u);
    CHECK(arm.control.reached()[0] == arm.control.solutions().size());
}

TEST_CASE("the arm ends the command at the distinct posture nearest where it stood", "[manipulator][multi-start]")
{
    standing_arm arm(searching());

    // One posture is the arm's own first joint value a full turn on and the other a fraction of a
    // turn away, so which the arm takes says the fold and the drive read the same wrapped distance.
    arm.control.solve_from_seeds(target_at(0.5), seeds_of({{0.9, 1.0}, {0.25 + 2.0 * std::numbers::pi, 1.0}}));

    REQUIRE(arm.control.solutions().size() == 2u);
    REQUIRE(played_out(arm.control));
    CHECK(is_approx_equal(arm.control.joint_positions(), configuration(0.25 + 2.0 * std::numbers::pi, -0.25), reached_at));
}
