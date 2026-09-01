#include "bent_manipulator.h"
#include "described_slots.h"

#include "praxis/manipulator/slots.h"
#include "praxis/manipulator/evaluation.h"
#include "praxis/manipulator/capabilities.h"

#include "praxis/trajectory/capabilities.h"

#include "praxis/evaluation/report.h"
#include "praxis/evaluation/residual.h"
#include "praxis/evaluation/slot_evaluation.h"

#include <catch2/catch_test_macros.hpp>

#include <span>
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
using fixture::described_slots;
using fixture::uncompared_slots;

std::vector<std::string_view> described_in_order(std::span<const capability_view> described)
{
    std::vector<std::string_view> named;

    for(const capability_view &view : described)
        for(const slot_descriptor &slot : view.slots())
            named.push_back(slot.name);

    return named;
}

std::vector<std::string_view> reported_in_order(const evaluation_report &reported)
{
    std::vector<std::string_view> named;

    for(const slot_report &slot : reported.slots)
        named.push_back(slot.slot);

    return named;
}

std::size_t carrying(std::span<const evaluation_view> compared, residual_kind kind)
{
    std::size_t seen = 0;

    for(const evaluation_view &view : compared)
        for(const slot_evaluation &slot : view.slots())
            seen += slot.kind == kind ? 1u : 0u;

    return seen;
}

evaluation_report reported_against(const manipulator::capabilities &reference, const manipulator::capabilities &other)
{
    return evaluate(manipulator::evaluation_views(reference, other), recorded_seed, cases_per_row);
}

bool names(const std::vector<std::string_view> &reported, std::string_view slot)
{
    return std::find(reported.begin(), reported.end(), slot) != reported.end();
}

// Every reported row stands where the descriptions put it, taken in order and with nothing between
// them reordered: a table comparing fewer slots than are described leaves that of the equality.
bool follows(const std::vector<std::string_view> &reported, const std::vector<std::string_view> &described)
{
    std::size_t at = 0;

    for(std::string_view named : described)
        if(at < reported.size() && reported[at] == named)
            ++at;

    return at == reported.size();
}

}

TEST_CASE("the_reference_against_itself_agrees_on_every_row_at_the_shipped_bounds")
{
    fixture::bend_every_row_by({});

    const manipulator::capabilities reference = manipulator::baseline();
    const evaluation_report reported          = reported_against(reference, reference);

    REQUIRE(reported.slots.size() == compared_slots);
    REQUIRE(reported.seed == recorded_seed);
    REQUIRE(reported.cases_per_slot == cases_per_row);
    for(const slot_report &slot : reported.slots)
    {
        INFO("row " << slot.slot);
        REQUIRE(slot.verdict == agreement::agreed);
    }
    REQUIRE(every_slot_agreed(reported));
    REQUIRE(disagreeing_slots(reported).empty());
}

TEST_CASE("the_report_lists_its_rows_in_the_capabilitys_own_enumerator_order_and_alike_on_a_repeated_run")
{
    fixture::bend_every_row_by({});

    const manipulator::capabilities reference = manipulator::baseline();
    const evaluation_report reported          = reported_against(reference, reference);
    const evaluation_report again             = reported_against(reference, reference);

    REQUIRE(follows(reported_in_order(reported), described_in_order(manipulator::capability_views(reference))));
    REQUIRE(reported_in_order(again) == reported_in_order(reported));
    REQUIRE(reported.slots.front().slot == "fk.forward_kinematics");
    REQUIRE(reported.slots.back().slot == "trajectory.task_space_waypoints");
    for(std::size_t row = 0; row < compared_slots; ++row)
    {
        REQUIRE(reported.slots[row].verdict == again.slots[row].verdict);
        REQUIRE(reported.slots[row].worst.magnitude == again.slots[row].worst.magnitude);
        REQUIRE(reported.slots[row].worst.linear_error_metres == again.slots[row].worst.linear_error_metres);
        REQUIRE(reported.slots[row].worst_case_index == again.slots[row].worst_case_index);
    }
}

TEST_CASE("a_row_whose_name_is_misspelled_is_reported_in_both_directions_rather_than_passing_unseen")
{
    const manipulator::capabilities reference = manipulator::baseline();
    const auto described                      = manipulator::capability_views(reference);
    const auto intact                         = manipulator::evaluation_views(reference, reference);
    std::vector<slot_evaluation> rows(intact[1].slots().begin(), intact[1].slots().end());

    rows.at(0).name = "dk.space_jacobean";

    const capability_evaluations<manipulator::differential_kinematics_ops> misspelled{"manipulator", rows};
    auto compared = intact;

    compared[1] = evaluation_view::of(reference.dk, reference.dk, misspelled);

    REQUIRE(unevaluated_slots(described, compared).size() == unevaluated_slots(described, intact).size() + 1u);
    REQUIRE(names(unevaluated_slots(described, compared), "dk.space_jacobian"));
    REQUIRE(unnamed_evaluations(described, compared).size() == 1u);
    REQUIRE(names(unnamed_evaluations(described, compared), "dk.space_jacobean"));
}

TEST_CASE("the_residual_kinds_are_distributed_over_the_rows_as_the_tables_assign_them")
{
    const manipulator::capabilities reference = manipulator::baseline();
    const auto compared                       = manipulator::evaluation_views(reference, reference);
    const std::size_t poses                   = carrying(compared, residual_kind::pose);
    const std::size_t elements                = carrying(compared, residual_kind::element_wise);
    const std::size_t angles                  = carrying(compared, residual_kind::geodesic);

    REQUIRE(poses == 11u);
    REQUIRE(elements == 5u);
    REQUIRE(angles == 1u);
    REQUIRE(carrying(compared, residual_kind::axis_up_to_sign) == 0u);
    REQUIRE(carrying(compared, residual_kind::log_up_to_branch) == 0u);
    REQUIRE(poses + elements + angles == compared_slots);
}

TEST_CASE("every_described_slot_but_the_analytic_solve_carries_a_comparator_and_that_one_is_named")
{
    const manipulator::capabilities reference                = manipulator::baseline();
    const auto described                                     = manipulator::capability_views(reference);
    const auto compared                                      = manipulator::evaluation_views(reference, reference);
    const std::vector<std::string_view> without_a_comparison = unevaluated_slots(described, compared);

    REQUIRE(described_in_order(described).size() == described_slots);
    REQUIRE(without_a_comparison.size() == uncompared_slots);
    REQUIRE(without_a_comparison.front() == "ik.analytic_inverse_kinematics");
    REQUIRE(unnamed_evaluations(described, compared).empty());
}

// The ledger matches a slot name byte for byte and reads nothing of which extension owns it, so the
// manipulator's task-space via-point factory and the configuration-space one the trajectory
// extension describes have to be spelled apart or one of them would answer for the other.
TEST_CASE("the_two_via_point_factories_named_under_trajectory_are_spelled_apart")
{
    const manipulator::capabilities arm       = manipulator::baseline();
    const trajectory::capabilities shapes     = trajectory::baseline();
    const std::vector<std::string_view> here  = described_in_order(manipulator::capability_views(arm));
    const std::vector<std::string_view> there = described_in_order(trajectory::capability_views(shapes));

    REQUIRE(names(here, "trajectory.task_space_waypoints"));
    REQUIRE(names(there, "trajectory.joint_space_waypoints"));
    REQUIRE_FALSE(names(here, "trajectory.joint_space_waypoints"));
    REQUIRE_FALSE(names(there, "trajectory.task_space_waypoints"));
}
