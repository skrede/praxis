#include "literal_pose_run.h"

#include "praxis/trajectory/evaluation.h"
#include "praxis/trajectory/capabilities.h"
#include "praxis/trajectory/pose_trajectory.h"

#include "praxis/trajectory/baseline/path.h"
#include "praxis/trajectory/baseline/time_scaling.h"
#include "praxis/trajectory/baseline/pose_trajectory.h"

#include "praxis/evaluation/report.h"
#include "praxis/evaluation/tolerance.h"
#include "praxis/evaluation/slot_evaluation.h"

#include "praxis/rigid_motion/frame.h"
#include "praxis/rigid_motion/screw.h"
#include "praxis/rigid_motion/capabilities.h"

#include <catch2/catch_test_macros.hpp>

#include <span>
#include <array>
#include <memory>
#include <cstddef>
#include <utility>

using namespace praxis;
using namespace praxis::trajectory;

namespace {

transform seed_pose()
{
    transform tf = transform::Identity();
    tf(0, 3)     = 0.25;
    tf(1, 3)     = -0.5;
    tf(2, 3)     = 1.75;
    return tf;
}

int quintic_calls   = 0;
int decoupled_calls = 0;

expected<scaling_sample, refusal> counted_quintic(double t, double duration)
{
    ++quintic_calls;
    return quintic(t, duration);
}

expected<transform, refusal> counted_decoupled(const transform &start, const transform &end, double s)
{
    ++decoupled_calls;
    return decoupled(start, end, s);
}

// The pose capability's view, in the aggregate's member order.
constexpr std::size_t pose_view = 2u;

std::unique_ptr<pose_trajectory_generator> inert_generator()
{
    pose_trajectory_ops ops{};
    auto motion = ops.decoupled_pose_waypoints(rigid_motion::screw_ops{}, rigid_motion::frame_ops{}, path_ops{}, time_scaling_ops{}, {}, seed_pose(), 1.0, 1.0);
    REQUIRE(motion);

    return std::move(*motion);
}

// Both pose rows at the default seed, the reference on one side and the binding given on the other.
evaluation::evaluation_report judged_against(const pose_trajectory_ops &bound)
{
    const capabilities reference = baseline();
    capabilities answering       = baseline();
    answering.pose_trajectory    = bound;

    const std::array views = evaluation_views(reference, answering);

    return evaluation::evaluate(std::span(views).subspan(pose_view, 1u));
}

}

TEST_CASE("the_pose_waypoint_slot_defaults_to_the_inert_factory")
{
    pose_trajectory_ops ops{};

    REQUIRE(ops.decoupled_pose_waypoints == &inert::decoupled_pose_waypoints);
    REQUIRE(ops.screw_pose_waypoints == &inert::screw_pose_waypoints);
    REQUIRE(inert_generator() != nullptr);
}

TEST_CASE("the_inert_pose_generator_reports_a_duration_of_zero")
{
    REQUIRE(is_approx_equal(inert_generator()->duration(), 0.0));
}

// An unbound slot holds no plan, so it answers no pose at all: reporting the seed would read as a
// motion that ran and arrived.
TEST_CASE("the_inert_pose_generator_refuses_to_sample_rather_than_reporting_the_seed")
{
    const std::unique_ptr<pose_trajectory_generator> motion = inert_generator();

    for(double t : std::array<double, 3>{0.0, 0.5, 12.0})
    {
        const expected<pose_sample, refusal> sampled = motion->sample(t);

        REQUIRE(!sampled);
        REQUIRE(sampled.error() == refusal::not_implemented);
    }
}

TEST_CASE("sampling_by_absolute_time_does_not_depend_on_the_order_the_times_arrive_in")
{
    const std::unique_ptr<pose_trajectory_generator> motion = inert_generator();
    const std::array<double, 4> times{0.0, 0.25, 0.75, 2.0};

    for(double t : times)
        REQUIRE(!motion->sample(t));

    for(std::size_t i = times.size(); i > 0; --i)
        REQUIRE(!motion->sample(times[i - 1]));
}

TEST_CASE("a_literal_pose_run_sweeping_the_path_and_the_scaling_it_is_handed_agrees_on_both_pose_rows")
{
    const evaluation::evaluation_report reported =
            judged_against(pose_trajectory_ops{&fixture::literal_pose_waypoints<&path_ops::decoupled>, &fixture::literal_pose_waypoints<&path_ops::screw>});

    REQUIRE(reported.slots.size() == 2u);
    for(const evaluation::slot_report &row : reported.slots)
    {
        INFO(row.slot);
        CHECK(row.verdict == evaluation::agreement::agreed);
    }
}

TEST_CASE("a_pose_run_sweeping_a_path_of_its_own_rather_than_the_one_it_is_handed_differs_on_both_pose_rows")
{
    const evaluation::evaluation_report reported =
            judged_against(pose_trajectory_ops{&fixture::own_path_pose_waypoints<&decoupled>, &fixture::own_path_pose_waypoints<&trajectory::screw>});

    REQUIRE(reported.slots.size() == 2u);
    for(const evaluation::slot_report &row : reported.slots)
    {
        INFO(row.slot);
        CHECK(row.verdict == evaluation::agreement::differed);
    }
}

TEST_CASE("a_pose_run_samples_through_the_time_scaling_and_the_path_it_is_handed")
{
    const rigid_motion::capabilities rigid = rigid_motion::baseline();
    time_scaling_ops scalings              = trajectory::baseline().time_scaling;
    path_ops paths                         = trajectory::baseline().path;
    scalings.quintic                       = &counted_quintic;
    paths.decoupled                        = &counted_decoupled;

    const std::array<transform, 1> via{transform::Identity()};
    const auto motion = trajectory::decoupled_pose_waypoints(rigid.screw, rigid.frame, paths, scalings, via, seed_pose(), 0.5, 0.5);
    REQUIRE(motion);

    const int scaled = quintic_calls;
    const int swept  = decoupled_calls;
    REQUIRE((*motion)->sample(0.25 * (*motion)->duration()));

    CHECK(quintic_calls > scaled);
    CHECK(decoupled_calls > swept);
}
