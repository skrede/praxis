#include "bent_generators.h"

#include "praxis/trajectory/evaluation.h"
#include "praxis/trajectory/capabilities.h"

#include "praxis/evaluation/report.h"
#include "praxis/evaluation/residual.h"
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

// The two pose rows are the seventh and eighth the report lists.
constexpr std::size_t first_pose_row = 6u;
constexpr std::size_t pose_rows      = 2u;

// One displacement per half, each far enough above the bound its half is judged at that no rounding
// of the fold reaches across.
constexpr double turned_by = 1.0e-2;
constexpr double moved_by  = 1.0e-1;

std::array<double, static_cast<std::size_t>(fixture::bent::count)> only(fixture::bent which, double by)
{
    std::array<double, static_cast<std::size_t>(fixture::bent::count)> one{};

    one.at(static_cast<std::size_t>(which)) = by;

    return one;
}

// Both pose rows against bindings displaced in one quantity alone, and the worst residual each row
// reported.
evaluation_report bent_in(fixture::bent which, double by)
{
    const trajectory::capabilities reference = trajectory::baseline();

    fixture::bend_every_row_by(only(which, by));

    trajectory::capabilities displaced                 = trajectory::baseline();
    displaced.pose_trajectory.decoupled_pose_waypoints = &fixture::displaced_decoupled_pose_waypoints;
    displaced.pose_trajectory.screw_pose_waypoints     = &fixture::displaced_screw_pose_waypoints;

    const evaluation_report reported = evaluate(trajectory::evaluation_views(reference, displaced), recorded_seed, cases_per_row);

    fixture::bend_every_row_by({});

    return reported;
}

bool close_to(double seen, double wanted)
{
    return std::abs(seen - wanted) <= 1.0e-3 * wanted;
}

}

TEST_CASE("a_displacement_of_a_sampled_rotation_moves_the_magnitude_half_and_leaves_the_linear_one_where_it_was")
{
    const evaluation_report reported = bent_in(fixture::bent::pose_radians, turned_by);

    REQUIRE(reported.slots.size() == first_pose_row + pose_rows + 1u);
    for(std::size_t row = first_pose_row; row < first_pose_row + pose_rows; ++row)
    {
        INFO(reported.slots.at(row).slot);
        REQUIRE(reported.slots.at(row).verdict == agreement::differed);
        REQUIRE(close_to(reported.slots.at(row).worst.magnitude, turned_by));
        REQUIRE(reported.slots.at(row).worst.linear_error_metres == 0.0);
    }
}

TEST_CASE("a_displacement_of_a_sampled_translation_moves_the_linear_half_and_leaves_the_magnitude_where_it_was")
{
    const evaluation_report reported = bent_in(fixture::bent::pose_metres, moved_by);

    for(std::size_t row = first_pose_row; row < first_pose_row + pose_rows; ++row)
    {
        INFO(reported.slots.at(row).slot);
        REQUIRE(reported.slots.at(row).verdict == agreement::differed);
        REQUIRE(reported.slots.at(row).worst.magnitude == 0.0);
        REQUIRE(close_to(reported.slots.at(row).worst.linear_error_metres, moved_by));
    }
}

TEST_CASE("a_displacement_of_a_sampled_twists_angular_part_is_reported_in_the_magnitude_half_alone")
{
    const evaluation_report reported = bent_in(fixture::bent::twist_radians, turned_by);

    for(std::size_t row = first_pose_row; row < first_pose_row + pose_rows; ++row)
    {
        INFO(reported.slots.at(row).slot);
        REQUIRE(reported.slots.at(row).verdict == agreement::differed);
        REQUIRE(close_to(reported.slots.at(row).worst.magnitude, turned_by));
        REQUIRE(reported.slots.at(row).worst.linear_error_metres == 0.0);
    }
}

TEST_CASE("a_displacement_of_a_sampled_twists_linear_part_is_reported_in_the_linear_half_alone")
{
    const evaluation_report reported = bent_in(fixture::bent::twist_metres, moved_by);

    for(std::size_t row = first_pose_row; row < first_pose_row + pose_rows; ++row)
    {
        INFO(reported.slots.at(row).slot);
        REQUIRE(reported.slots.at(row).verdict == agreement::differed);
        REQUIRE(reported.slots.at(row).worst.magnitude == 0.0);
        REQUIRE(close_to(reported.slots.at(row).worst.linear_error_metres, moved_by));
    }
}

TEST_CASE("the_two_pose_rows_are_drawn_over_runs_whose_motion_parts_the_two_task_space_paths")
{
    const trajectory::capabilities reference = trajectory::baseline();
    trajectory::capabilities exchanged       = trajectory::baseline();

    exchanged.pose_trajectory.decoupled_pose_waypoints = &trajectory::screw_pose_waypoints;
    exchanged.pose_trajectory.screw_pose_waypoints     = &trajectory::decoupled_pose_waypoints;

    const evaluation_report reported = evaluate(trajectory::evaluation_views(reference, exchanged), recorded_seed, cases_per_row);

    for(std::size_t row = first_pose_row; row < first_pose_row + pose_rows; ++row)
    {
        INFO(reported.slots.at(row).slot);
        REQUIRE(reported.slots.at(row).verdict == agreement::differed);
    }
}
