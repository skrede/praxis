#include "praxis/trajectory/slots.h"
#include "praxis/trajectory/evaluation.h"
#include "praxis/trajectory/capabilities.h"

#include "praxis/evaluation/report.h"
#include "praxis/evaluation/residual.h"
#include "praxis/evaluation/slot_evaluation.h"

#include <catch2/catch_test_macros.hpp>

#include <set>
#include <array>
#include <string>
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

std::size_t every_compared_slot()
{
    return static_cast<std::size_t>(trajectory::time_scaling_slot::count) + static_cast<std::size_t>(trajectory::path_slot::count) +
            static_cast<std::size_t>(trajectory::pose_trajectory_slot::count) + static_cast<std::size_t>(trajectory::trajectory_slot::count);
}

std::vector<std::string_view> described_in_order(const std::array<capability_view, 4> &described)
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

std::size_t carrying(const std::array<evaluation_view, 4> &compared, residual_kind kind)
{
    std::size_t seen = 0;

    for(const evaluation_view &view : compared)
        for(const slot_evaluation &slot : view.slots())
            seen += slot.kind == kind ? 1u : 0u;

    return seen;
}

bool names(const std::vector<std::string_view> &reported, std::string_view slot)
{
    return std::find(reported.begin(), reported.end(), slot) != reported.end();
}

}

TEST_CASE("the_reference_compared_with_itself_agrees_on_every_row_this_extension_carries")
{
    const trajectory::capabilities reference      = trajectory::baseline();
    const std::array<evaluation_view, 4> compared = trajectory::evaluation_views(reference, reference);
    const evaluation_report reported              = evaluate(compared, recorded_seed, cases_per_row);

    REQUIRE(reported.slots.size() == every_compared_slot());
    REQUIRE(every_slot_agreed(reported));
}

TEST_CASE("no_slot_this_extension_describes_is_left_uncompared_and_no_row_names_a_slot_no_descriptor_does")
{
    const trajectory::capabilities reference       = trajectory::baseline();
    const std::array<capability_view, 4> described = trajectory::capability_views(reference);
    const std::array<evaluation_view, 4> compared  = trajectory::evaluation_views(reference, reference);

    REQUIRE(described_in_order(described).size() == every_compared_slot());
    REQUIRE(evaluate(compared, recorded_seed, 0).slots.size() == every_compared_slot());
    REQUIRE(unevaluated_slots(described, compared).empty());
    REQUIRE(unnamed_evaluations(described, compared).empty());
}

TEST_CASE("every_row_is_named_once_and_no_name_is_empty")
{
    const trajectory::capabilities reference      = trajectory::baseline();
    const std::array<evaluation_view, 4> compared = trajectory::evaluation_views(reference, reference);
    std::set<std::string> once;

    for(std::string_view name : reported_in_order(evaluate(compared, recorded_seed, 0)))
    {
        REQUIRE_FALSE(name.empty());
        REQUIRE(once.insert(std::string(name)).second);
    }
    REQUIRE(once.size() == every_compared_slot());
}

TEST_CASE("each_tables_row_count_is_the_count_its_own_slot_enumeration_names")
{
    const trajectory::capabilities reference      = trajectory::baseline();
    const std::array<evaluation_view, 4> compared = trajectory::evaluation_views(reference, reference);

    REQUIRE(compared.at(0).slots().size() == static_cast<std::size_t>(trajectory::time_scaling_slot::count));
    REQUIRE(compared.at(1).slots().size() == static_cast<std::size_t>(trajectory::path_slot::count));
    REQUIRE(compared.at(2).slots().size() == static_cast<std::size_t>(trajectory::pose_trajectory_slot::count));
    REQUIRE(compared.at(3).slots().size() == static_cast<std::size_t>(trajectory::trajectory_slot::count));
}

TEST_CASE("the_report_lists_its_rows_in_the_capabilitys_own_enumerator_order_and_alike_on_every_run")
{
    const trajectory::capabilities reference       = trajectory::baseline();
    const std::array<capability_view, 4> described = trajectory::capability_views(reference);
    const std::array<evaluation_view, 4> compared  = trajectory::evaluation_views(reference, reference);
    const evaluation_report reported               = evaluate(compared, recorded_seed, 4);
    const evaluation_report again                  = evaluate(compared, recorded_seed, 4);

    REQUIRE(reported_in_order(reported) == described_in_order(described));
    REQUIRE(reported_in_order(again) == reported_in_order(reported));
    REQUIRE(reported.slots.front().slot == "time_scaling.cubic");
    REQUIRE(reported.slots.back().slot == "trajectory.joint_space_waypoints");
}

TEST_CASE("the_residual_kinds_are_distributed_over_the_rows_as_the_four_tables_assign_them")
{
    const trajectory::capabilities reference      = trajectory::baseline();
    const std::array<evaluation_view, 4> compared = trajectory::evaluation_views(reference, reference);
    const std::size_t elements                    = carrying(compared, residual_kind::element_wise);
    const std::size_t poses                       = carrying(compared, residual_kind::pose);

    REQUIRE(elements == 5u);
    REQUIRE(poses == 4u);
    REQUIRE(carrying(compared, residual_kind::geodesic) == 0u);
    REQUIRE(carrying(compared, residual_kind::axis_up_to_sign) == 0u);
    REQUIRE(carrying(compared, residual_kind::log_up_to_branch) == 0u);
    REQUIRE(elements + poses == every_compared_slot());
}

TEST_CASE("a_row_whose_name_differs_from_the_descriptors_by_one_character_is_reported_in_both_directions")
{
    const trajectory::capabilities reference       = trajectory::baseline();
    const std::array<capability_view, 4> described = trajectory::capability_views(reference);
    const std::array<evaluation_view, 4> intact    = trajectory::evaluation_views(reference, reference);
    std::vector<slot_evaluation> rows(intact.front().slots().begin(), intact.front().slots().end());

    rows.at(1).name = "time_scaling.Quintic";

    const capability_evaluations<trajectory::time_scaling_ops> misspelled{"trajectory", rows};
    const std::array<evaluation_view, 4> compared{evaluation_view::of(reference.time_scaling, reference.time_scaling, misspelled), intact.at(1), intact.at(2), intact.at(3)};

    REQUIRE(unevaluated_slots(described, compared).size() == 1u);
    REQUIRE(names(unevaluated_slots(described, compared), "time_scaling.quintic"));
    REQUIRE(unnamed_evaluations(described, compared).size() == 1u);
    REQUIRE(names(unnamed_evaluations(described, compared), "time_scaling.Quintic"));
}
