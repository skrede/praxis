#include "praxis/evaluation/report.h"
#include "praxis/evaluation/residual.h"
#include "praxis/evaluation/slot_evaluation.h"

#include "praxis/rigid_motion/slots.h"
#include "praxis/rigid_motion/evaluation.h"
#include "praxis/rigid_motion/capabilities.h"

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

std::size_t every_slot()
{
    return static_cast<std::size_t>(rigid_motion::frame_slot::count) + static_cast<std::size_t>(rigid_motion::screw_slot::count);
}

std::vector<std::string_view> described_in_order(const std::array<capability_view, 2> &described)
{
    std::vector<std::string_view> named;

    for(const capability_view &view : described)
        for(const slot_descriptor &slot : view.slots())
            named.push_back(slot.name);

    return named;
}

std::vector<std::string_view> evaluated_in_order(const std::array<evaluation_view, 2> &compared)
{
    std::vector<std::string_view> named;

    for(const evaluation_view &view : compared)
        for(const slot_evaluation &slot : view.slots())
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

std::size_t carrying(const std::array<evaluation_view, 2> &compared, residual_kind kind)
{
    std::size_t seen = 0;

    for(const evaluation_view &view : compared)
        for(const slot_evaluation &slot : view.slots())
            if(slot.kind == kind)
                ++seen;

    return seen;
}

bool names(const std::vector<std::string_view> &reported, std::string_view slot)
{
    return std::find(reported.begin(), reported.end(), slot) != reported.end();
}

}

TEST_CASE("every_slot_the_seam_declares_carries_an_evaluation")
{
    const rigid_motion::capabilities reference     = rigid_motion::baseline();
    const std::array<capability_view, 2> described = rigid_motion::capability_views(reference);
    const std::array<evaluation_view, 2> compared  = rigid_motion::evaluation_views(reference, reference);

    REQUIRE(unevaluated_slots(described, compared).empty());
}

TEST_CASE("every_evaluation_names_a_slot_some_describing_table_declares")
{
    const rigid_motion::capabilities reference     = rigid_motion::baseline();
    const std::array<capability_view, 2> described = rigid_motion::capability_views(reference);
    const std::array<evaluation_view, 2> compared  = rigid_motion::evaluation_views(reference, reference);

    REQUIRE(unnamed_evaluations(described, compared).empty());
}

TEST_CASE("the_two_name_sets_are_the_same_size_and_every_name_is_non_empty_and_appears_once")
{
    const rigid_motion::capabilities reference     = rigid_motion::baseline();
    const std::array<capability_view, 2> described = rigid_motion::capability_views(reference);
    const std::array<evaluation_view, 2> compared  = rigid_motion::evaluation_views(reference, reference);
    const std::vector<std::string_view> declared   = described_in_order(described);
    const std::vector<std::string_view> carried    = evaluated_in_order(compared);
    std::set<std::string> once;

    REQUIRE(declared.size() == every_slot());
    REQUIRE(carried.size() == declared.size());
    for(std::string_view name : carried)
    {
        REQUIRE_FALSE(name.empty());
        REQUIRE(once.insert(std::string(name)).second);
    }
    REQUIRE(once.size() == every_slot());
}

TEST_CASE("the_five_residual_kinds_are_distributed_over_the_rows_as_the_tables_assign_them")
{
    const rigid_motion::capabilities reference    = rigid_motion::baseline();
    const std::array<evaluation_view, 2> compared = rigid_motion::evaluation_views(reference, reference);
    const std::size_t elements                    = carrying(compared, residual_kind::element_wise);
    const std::size_t angles                      = carrying(compared, residual_kind::geodesic);
    const std::size_t poses                       = carrying(compared, residual_kind::pose);
    const std::size_t axes                        = carrying(compared, residual_kind::axis_up_to_sign);
    const std::size_t logarithms                  = carrying(compared, residual_kind::log_up_to_branch);

    REQUIRE(elements == 9u);
    REQUIRE(angles == 8u);
    REQUIRE(poses == 6u);
    REQUIRE(axes == 2u);
    REQUIRE(logarithms == 4u);
    REQUIRE(elements + angles + poses + axes + logarithms == every_slot());
}

TEST_CASE("the_report_lists_its_slots_in_the_capabilitys_own_enumerator_order_and_alike_on_every_run")
{
    const rigid_motion::capabilities reference     = rigid_motion::baseline();
    const std::array<capability_view, 2> described = rigid_motion::capability_views(reference);
    const std::array<evaluation_view, 2> compared  = rigid_motion::evaluation_views(reference, reference);
    const evaluation_report reported               = evaluate(compared, recorded_seed, 4);
    const evaluation_report again                  = evaluate(compared, recorded_seed, 4);

    REQUIRE(reported_in_order(reported) == described_in_order(described));
    REQUIRE(reported_in_order(again) == reported_in_order(reported));
    REQUIRE(reported.slots.front().slot == "frame.euler_from_rotation_matrix");
    REQUIRE(reported.slots.back().slot == "screw.matrix_logarithm_se3");
}

TEST_CASE("a_row_whose_name_is_misspelled_is_reported_in_both_directions_rather_than_passing_unseen")
{
    const rigid_motion::capabilities reference     = rigid_motion::baseline();
    const std::array<capability_view, 2> described = rigid_motion::capability_views(reference);
    const std::array<evaluation_view, 2> intact    = rigid_motion::evaluation_views(reference, reference);
    std::vector<slot_evaluation> rows(intact.front().slots().begin(), intact.front().slots().end());

    rows.at(3).name = "frame.rotate_zed";

    const capability_evaluations<rigid_motion::frame_ops> misspelled{"rigid_motion", rows};
    const capability_evaluations<rigid_motion::screw_ops> untouched{"rigid_motion", intact.back().slots()};
    const std::array<evaluation_view, 2> compared{evaluation_view::of(reference.frame, reference.frame, misspelled), evaluation_view::of(reference.screw, reference.screw, untouched)};

    REQUIRE(unevaluated_slots(described, compared).size() == 1u);
    REQUIRE(names(unevaluated_slots(described, compared), "frame.rotate_z"));
    REQUIRE(unnamed_evaluations(described, compared).size() == 1u);
    REQUIRE(names(unnamed_evaluations(described, compared), "frame.rotate_zed"));
}
