#include "bent_manipulator.h"
#include "described_slots.h"

#include "praxis/manipulator/evaluation.h"
#include "praxis/manipulator/capabilities.h"

#include "praxis/evaluation/report.h"
#include "praxis/evaluation/slot_evaluation.h"

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <algorithm>
#include <string_view>

using namespace praxis;
using namespace praxis::evaluation;

namespace {

constexpr std::uint64_t recorded_seed = 0xC0FFEEu;
constexpr std::size_t cases_per_row   = 24u;
using fixture::compared_slots;

// A displacement a decade above the loosest bound the rows of its kind are judged at, and one a
// decade under the tightest of them. Both are written out rather than derived from those bounds, so a
// bound moved up onto the first or down onto the second fails here. A displacement of every joint
// travels to the pose a solve row is read at along the chain's lever arms and arrives amplified, so
// that entry's pair is read off the displacement that crosses the bound rather than off the bound.
// The last entry's row reports the displacement itself, so its pair stands three times either side
// of the bound instead of a decade: a case standing on a bound the comparison is inclusive at decides
// nothing.
constexpr std::array<double, static_cast<std::size_t>(fixture::bent::count)> above_the_bound{1.0e-11, 1.0e-12, 1.0e-4, 1.0e-3, 3.0e-6, 3.0e-7};
constexpr std::array<double, static_cast<std::size_t>(fixture::bent::count)> beneath_the_bound{1.0e-14, 1.0e-14, 1.0e-14, 1.0e-13, 5.0e-10, 3.0e-8};

struct quiet_solve
{
    std::string_view slot;
    double displacement;
};

// Each solve row crosses its bound at a displacement of its own, so each holds a quiet entry a decade
// under that crossing; the kind's entry in the array above is the smallest of them.
constexpr std::array<quiet_solve, 5> beneath_each_solve{quiet_solve{"ik.inverse_kinematics", 2.0e-8}, quiet_solve{"robot.ik_solve_pose", 2.0e-9},
                                                        quiet_solve{"robot.ik_solve_flange_pose", 2.0e-9}, quiet_solve{"motion.task_space_pose", 5.0e-10},
                                                        quiet_solve{"motion.task_space_screw", 5.0e-9}};

std::vector<std::string_view> rows_one_bend_names(const evaluation_report &reported, fixture::bent which)
{
    std::vector<std::string_view> named;

    for(const slot_report &slot : reported.slots)
        if(fixture::is_bent(which, slot.slot))
            named.push_back(slot.slot);

    return named;
}

evaluation_report reported_against(const manipulator::capabilities &reference, const manipulator::capabilities &other)
{
    return evaluate(manipulator::evaluation_views(reference, other), recorded_seed, cases_per_row);
}

}

TEST_CASE("each_bend_in_turn_is_named_by_the_ledger_and_no_other_row_is")
{
    const manipulator::capabilities reference = manipulator::baseline();

    for(std::size_t which = 0; which < static_cast<std::size_t>(fixture::bent::count); ++which)
    {
        std::array<double, static_cast<std::size_t>(fixture::bent::count)> one{};
        one[which] = above_the_bound[which];

        fixture::bend_every_row_by(one);

        const evaluation_report reported = reported_against(reference, fixture::bent_everywhere());

        fixture::bend_every_row_by({});

        INFO("bend " << which);
        REQUIRE_FALSE(every_slot_agreed(reported));
        REQUIRE(disagreeing_slots(reported) == rows_one_bend_names(reported, static_cast<fixture::bent>(which)));
        REQUIRE_FALSE(disagreeing_slots(reported).empty());
    }
}

TEST_CASE("a_displacement_a_decade_beneath_every_bound_is_reported_by_no_row_at_all")
{
    const manipulator::capabilities reference = manipulator::baseline();

    fixture::bend_every_row_by(beneath_the_bound);

    const evaluation_report quiet = reported_against(reference, fixture::bent_everywhere());

    fixture::bend_every_row_by(above_the_bound);

    const evaluation_report loud = reported_against(reference, fixture::bent_everywhere());

    fixture::bend_every_row_by({});

    REQUIRE(every_slot_agreed(quiet));
    REQUIRE(disagreeing_slots(quiet).empty());
    REQUIRE(disagreeing_slots(loud).size() == compared_slots);
}

TEST_CASE("a_configuration_displacement_a_decade_beneath_a_solve_rows_own_crossing_is_not_reported_by_that_row")
{
    const manipulator::capabilities reference = manipulator::baseline();

    for(const quiet_solve &row : beneath_each_solve)
    {
        std::array<double, static_cast<std::size_t>(fixture::bent::count)> one{};
        one[static_cast<std::size_t>(fixture::bent::configuration)] = row.displacement;

        fixture::bend_every_row_by(one);

        const evaluation_report reported = reported_against(reference, fixture::bent_everywhere());

        fixture::bend_every_row_by({});

        const std::vector<std::string_view> solves = rows_one_bend_names(reported, fixture::bent::configuration);
        const std::vector<std::string_view> named  = disagreeing_slots(reported);

        INFO(row.slot);
        REQUIRE(solves.size() == beneath_each_solve.size());
        REQUIRE(std::ranges::find(solves, row.slot) != solves.end());
        CHECK(std::ranges::find(named, row.slot) == named.end());
    }
}
