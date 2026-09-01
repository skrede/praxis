#include "driven_generators.h"

#include "praxis/trajectory/slots.h"
#include "praxis/trajectory/evaluation.h"
#include "praxis/trajectory/capabilities.h"

#include "praxis/evaluation/report.h"
#include "praxis/evaluation/residual.h"
#include "praxis/evaluation/generation.h"
#include "praxis/evaluation/slot_evaluation.h"

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>

using namespace praxis;
using namespace praxis::evaluation;

namespace {

constexpr std::uint64_t recorded_seed = 0xC0FFEEu;
constexpr std::size_t cases_per_row   = 24u;
constexpr std::size_t wider_run       = 200u;

// The rows answering a prepared generator are the last three the report lists.
constexpr std::size_t first_generator_row = 6u;
constexpr std::size_t generator_rows      = 3u;

evaluation_report reported_against(const trajectory::capabilities &reference, const trajectory::capabilities &other, std::size_t cases)
{
    return evaluate(trajectory::evaluation_views(reference, other), recorded_seed, cases);
}

trajectory::capabilities declining_every_generator()
{
    trajectory::capabilities shapes = trajectory::baseline();

    shapes.trajectory.joint_space_waypoints         = &fixture::declining_joint_space_waypoints;
    shapes.pose_trajectory.decoupled_pose_waypoints = &fixture::declining_pose_waypoints;
    shapes.pose_trajectory.screw_pose_waypoints     = &fixture::declining_pose_waypoints;

    return shapes;
}

trajectory::capabilities building_no_generator()
{
    trajectory::capabilities shapes = trajectory::baseline();

    shapes.trajectory.joint_space_waypoints         = &fixture::unbuilt_joint_space_waypoints;
    shapes.pose_trajectory.decoupled_pose_waypoints = &fixture::unbuilt_pose_waypoints;
    shapes.pose_trajectory.screw_pose_waypoints     = &fixture::unbuilt_pose_waypoints;

    return shapes;
}

// The row over `trajectory.joint_space_waypoints`, driven directly so a case can be handed a source
// of a spread the published run never draws from.
case_result one_case(const trajectory::capabilities &reference, spread drawn_from, std::size_t index)
{
    const std::array<evaluation_view, 4> compared = trajectory::evaluation_views(reference, reference);
    const slot_evaluation &row                    = compared.at(3).slots().front();
    case_source drawn                             = case_source::at_case(recorded_seed, row.name, drawn_from, index);

    return row.compare(compared.at(3).first(), compared.at(3).second(), drawn, row.allowed);
}

}

TEST_CASE("a_generator_pair_exactly_one_side_built_is_one_refused_and_the_side_that_built_one_is_never_driven")
{
    trajectory::capabilities counted         = trajectory::baseline();
    counted.trajectory.joint_space_waypoints = &fixture::counted_joint_space_waypoints;
    trajectory::capabilities unbuilt         = trajectory::baseline();
    unbuilt.trajectory.joint_space_waypoints = &fixture::unbuilt_joint_space_waypoints;

    fixture::forget_what_was_driven();

    const evaluation_report reported = reported_against(counted, unbuilt, cases_per_row);
    const slot_report &driving       = reported.slots.back();

    REQUIRE(reported.slots.size() == first_generator_row + generator_rows);
    REQUIRE(driving.slot == "trajectory.joint_space_waypoints");
    REQUIRE(driving.verdict == agreement::not_exercised);
    REQUIRE(driving.outcomes.one_refused == cases_per_row);
    REQUIRE(driving.outcomes.agreed == 0u);
    REQUIRE(driving.outcomes.differed == 0u);
    REQUIRE(driving.worst.magnitude == 0.0);
    REQUIRE(driving.worst.linear_error_metres == 0.0);
    REQUIRE(fixture::durations_read == 0u);
    REQUIRE(fixture::samples_taken == 0u);
}

TEST_CASE("two_generators_whose_durations_differ_are_reported_as_differing_and_are_never_sampled")
{
    trajectory::capabilities counted         = trajectory::baseline();
    counted.trajectory.joint_space_waypoints = &fixture::counted_joint_space_waypoints;

    trajectory::capabilities stretched         = trajectory::baseline();
    stretched.trajectory.joint_space_waypoints = &fixture::stretched_joint_space_waypoints;

    fixture::forget_what_was_driven();

    const evaluation_report reported = reported_against(counted, stretched, cases_per_row);
    const slot_report &driving       = reported.slots.back();

    REQUIRE(driving.slot == "trajectory.joint_space_waypoints");
    REQUIRE(driving.verdict == agreement::differed);
    REQUIRE(driving.outcomes.differed == cases_per_row);
    REQUIRE(std::isinf(driving.worst.magnitude));
    REQUIRE(driving.worst.magnitude > 0.0);
    REQUIRE(fixture::durations_read == 2u * cases_per_row);
    REQUIRE(fixture::samples_taken == 0u);
}

TEST_CASE("two_generators_reporting_one_duration_are_sampled_at_a_count_that_does_not_follow_the_span")
{
    trajectory::capabilities counted         = trajectory::baseline();
    counted.trajectory.joint_space_waypoints = &fixture::counted_joint_space_waypoints;

    fixture::forget_what_was_driven();

    const evaluation_report once      = reported_against(counted, counted, cases_per_row);
    const std::size_t over_one_run    = fixture::samples_taken;
    const std::size_t durations_taken = fixture::durations_read;

    fixture::forget_what_was_driven();

    const evaluation_report twice = reported_against(counted, counted, 2u * cases_per_row);

    REQUIRE(once.slots.back().verdict == agreement::agreed);
    REQUIRE(once.slots.back().outcomes.agreed == cases_per_row);
    REQUIRE(twice.slots.back().outcomes.agreed == 2u * cases_per_row);
    REQUIRE(durations_taken == 2u * cases_per_row);
    REQUIRE(over_one_run > 0u);
    REQUIRE(over_one_run % (2u * cases_per_row) == 0u);
    REQUIRE(fixture::samples_taken == 2u * over_one_run);
}

TEST_CASE("a_generator_row_reaches_its_refusal_channel_through_the_object_it_was_handed_and_not_through_the_factory")
{
    const trajectory::capabilities inert{};
    const evaluation_report alike     = reported_against(inert, inert, cases_per_row);
    const evaluation_report differing = reported_against(inert, declining_every_generator(), cases_per_row);
    const evaluation_report one_sided = reported_against(building_no_generator(), inert, cases_per_row);

    REQUIRE(alike.slots.size() == first_generator_row + generator_rows);
    for(std::size_t row = first_generator_row; row < alike.slots.size(); ++row)
    {
        INFO(alike.slots.at(row).slot);
        REQUIRE(alike.slots.at(row).verdict == agreement::both_refused);
        REQUIRE(differing.slots.at(row).verdict == agreement::refused_differently);
        REQUIRE(one_sided.slots.at(row).verdict == agreement::not_exercised);
        REQUIRE(one_sided.slots.at(row).outcomes.one_refused == cases_per_row);
        REQUIRE(differing.slots.at(row).worst.magnitude == 0.0);
        REQUIRE(one_sided.slots.at(row).worst.magnitude == 0.0);
    }
}

TEST_CASE("a_run_the_harness_could_not_build_is_unusable_and_the_published_draw_reaches_no_such_run")
{
    const trajectory::capabilities reference = trajectory::baseline();
    const evaluation_report reported         = reported_against(reference, reference, wider_run);

    REQUIRE(one_case(reference, spread::near_singular, 0).verdict == agreement::unusable);
    REQUIRE(one_case(reference, spread::near_singular, 1).verdict == agreement::unusable);
    REQUIRE(one_case(reference, spread::bulk, 0).verdict == agreement::agreed);

    for(std::size_t row = first_generator_row; row < reported.slots.size(); ++row)
    {
        INFO(reported.slots.at(row).slot);
        REQUIRE(reported.slots.at(row).cases == wider_run);
        REQUIRE(reported.slots.at(row).outcomes.unusable == 0u);
        REQUIRE(reported.slots.at(row).outcomes.agreed == wider_run);
    }
}
