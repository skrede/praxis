#include "bent_generators.h"
#include "end_bent_trajectory.h"

#include "praxis/trajectory/evaluation.h"
#include "praxis/trajectory/capabilities.h"

#include "praxis/evaluation/report.h"
#include "praxis/evaluation/residual.h"
#include "praxis/evaluation/slot_evaluation.h"

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <string_view>

using namespace praxis;
using namespace praxis::evaluation;

namespace {

constexpr std::uint64_t recorded_seed = 0xC0FFEEu;
constexpr std::size_t cases_per_row   = 24u;
constexpr std::size_t compared_rows   = 9u;

using displacement = std::array<double, static_cast<std::size_t>(fixture::bent::count)>;

// One row, and one displacement per quantity a single one of that row's bounds is read in.
struct judged_at
{
    std::size_t row;
    displacement at;
};

// One entry per bound the nine rows carry, standing exactly at it. A bound is a HALF and not a row:
// a pose row is judged on its rotation in radians and its translation in metres separately, so a
// displacement standing at one of them leaves the other at zero, or a half widened by a decade
// hides behind whichever of the two the fold happened to report. Every number is written out rather
// than derived from the bounds themselves, so a bound moved a decade in either direction fails here.
constexpr std::array<judged_at, 13> every_bound{
        judged_at{0u, {1.0e-12, 0.0, 0.0, 0.0, 0.0}},   judged_at{1u, {1.0e-11, 0.0, 0.0, 0.0, 0.0}},   judged_at{2u, {1.0e-12, 0.0, 0.0, 0.0, 0.0}},
        judged_at{3u, {1.0e-13, 0.0, 0.0, 0.0, 0.0}},   judged_at{4u, {0.0, 1.0e-13, 0.0, 0.0, 0.0}},   judged_at{4u, {0.0, 0.0, 1.0e-12, 0.0, 0.0}},
        judged_at{5u, {0.0, 1.0e-13, 0.0, 0.0, 0.0}},   judged_at{5u, {0.0, 0.0, 1.0e-12, 0.0, 0.0}},   judged_at{6u, {0.0, 1.0e-3, 0.0, 1.0e-3, 0.0}},
        judged_at{6u, {0.0, 0.0, 1.0e-2, 0.0, 1.0e-2}}, judged_at{7u, {0.0, 1.0e-3, 0.0, 1.0e-3, 0.0}}, judged_at{7u, {0.0, 0.0, 1.0e-2, 0.0, 1.0e-2}},
        judged_at{8u, {1.0e-4, 0.0, 0.0, 0.0, 0.0}},
};

// The same displacements a decade beneath, which the row must fail to report.
displacement a_decade_beneath(const displacement &at)
{
    displacement under{};

    for(std::size_t which = 0; which < at.size(); ++which)
        under.at(which) = at.at(which) / 10.0;

    return under;
}

// A displacement no row is judged as agreeing at, for the case below that asks what a row reported
// rather than whether it reported anything.
constexpr displacement well_above_every_bound{1.0e-1, 1.0e-1, 1.0e-1, 1.0e-1, 1.0e-1};

evaluation_report reported_against(const trajectory::capabilities &reference, const trajectory::capabilities &other)
{
    return evaluate(trajectory::evaluation_views(reference, other), recorded_seed, cases_per_row);
}

evaluation_report over_paths(const trajectory::capabilities &reference, const trajectory::capabilities &bent, const capability_evaluations<trajectory::path_ops> &rows)
{
    const std::array<evaluation_view, 1> compared{evaluation_view::of(reference.path, bent.path, rows)};

    return evaluate(compared, recorded_seed, cases_per_row);
}

bool same(const slot_report &held, const slot_report &against)
{
    return held.slot == against.slot && held.verdict == against.verdict && held.cases == against.cases && held.worst_case_index == against.worst_case_index &&
            held.worst.kind == against.worst.kind && held.worst.magnitude == against.worst.magnitude && held.worst.linear_error_metres == against.worst.linear_error_metres;
}

}

TEST_CASE("a_displacement_standing_at_one_of_a_rows_own_bounds_is_reported_by_that_row_and_no_other_row_stirs")
{
    const trajectory::capabilities reference = trajectory::baseline();

    for(const judged_at &bound : every_bound)
    {
        fixture::bend_every_row_by(bound.at);

        const evaluation_report reported = reported_against(reference, fixture::bent_at(bound.row));

        fixture::bend_every_row_by({});

        INFO("row " << bound.row);
        REQUIRE(reported.slots.size() == compared_rows);
        REQUIRE(disagreeing_slots(reported) == std::vector<std::string_view>{reported.slots.at(bound.row).slot});
    }
}

TEST_CASE("a_displacement_a_decade_beneath_one_of_a_rows_own_bounds_is_reported_by_no_row_at_all")
{
    const trajectory::capabilities reference = trajectory::baseline();

    for(const judged_at &bound : every_bound)
    {
        fixture::bend_every_row_by(a_decade_beneath(bound.at));

        const evaluation_report quiet = reported_against(reference, fixture::bent_at(bound.row));

        fixture::bend_every_row_by({});

        INFO("row " << bound.row);
        REQUIRE(every_slot_agreed(quiet));
        REQUIRE(disagreeing_slots(quiet).empty());
    }
}

TEST_CASE("the_endpoints_a_task_space_row_draws_part_the_screw_path_from_the_decoupled_one")
{
    const trajectory::capabilities reference = trajectory::baseline();
    trajectory::capabilities exchanged       = trajectory::baseline();

    exchanged.path.screw     = &trajectory::decoupled;
    exchanged.path.decoupled = &trajectory::screw;

    const evaluation_report reported = reported_against(reference, exchanged);

    REQUIRE(disagreeing_slots(reported) == std::vector<std::string_view>{"path.screw", "path.decoupled"});
}

TEST_CASE("a_rows_inputs_do_not_move_when_the_table_around_it_is_cut_down_or_turned_around")
{
    const trajectory::capabilities reference = trajectory::baseline();

    fixture::bend_every_row_by(well_above_every_bound);

    const trajectory::capabilities bent        = fixture::bent_everywhere();
    const std::array<evaluation_view, 4> whole = trajectory::evaluation_views(reference, bent);
    const evaluation_report full               = evaluate(whole, recorded_seed, cases_per_row);
    const std::vector<slot_evaluation> rows(whole.at(1).slots().begin(), whole.at(1).slots().end());
    const std::vector<slot_evaluation> alone{rows.back()};
    const std::vector<slot_evaluation> turned_around(rows.rbegin(), rows.rend());
    const evaluation_report apart = over_paths(reference, bent, {"trajectory", alone});
    const evaluation_report other = over_paths(reference, bent, {"trajectory", turned_around});

    fixture::bend_every_row_by({});

    REQUIRE(full.slots.at(5).slot == "path.decoupled");
    REQUIRE(same(apart.slots.front(), full.slots.at(5)));
    REQUIRE(same(other.slots.front(), full.slots.at(5)));
}

TEST_CASE("the_times_a_case_samples_at_reach_both_ends_of_the_motion")
{
    const trajectory::capabilities reference = trajectory::baseline();

    trajectory::capabilities at_the_start         = trajectory::baseline();
    at_the_start.trajectory.joint_space_waypoints = &fixture::bent_at_the_start;

    trajectory::capabilities at_the_finish         = trajectory::baseline();
    at_the_finish.trajectory.joint_space_waypoints = &fixture::bent_at_the_finish;

    const evaluation_report early = reported_against(reference, at_the_start);
    const evaluation_report late  = reported_against(reference, at_the_finish);

    REQUIRE(early.slots.back().slot == "trajectory.joint_space_waypoints");
    REQUIRE(disagreeing_slots(early) == std::vector<std::string_view>{"trajectory.joint_space_waypoints"});
    REQUIRE(disagreeing_slots(late) == std::vector<std::string_view>{"trajectory.joint_space_waypoints"});
    REQUIRE(early.slots.back().outcomes.differed == cases_per_row);
    REQUIRE(late.slots.back().outcomes.differed == cases_per_row);
}
