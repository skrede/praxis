#include "bent_rigid_motion.h"

#include "praxis/evaluation/report.h"
#include "praxis/evaluation/residual.h"
#include "praxis/evaluation/generation.h"
#include "praxis/evaluation/slot_evaluation.h"

#include "praxis/rigid_motion/slots.h"
#include "praxis/rigid_motion/types.h"
#include "praxis/rigid_motion/evaluation.h"
#include "praxis/rigid_motion/capabilities.h"

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <vector>
#include <cstddef>
#include <optional>
#include <algorithm>
#include <string_view>
#include <type_traits>

using namespace praxis;
using namespace praxis::evaluation;

namespace {

// Enough draws for a bend that only shows on part of a role's range to show on one of them; the
// facility's own run is a thousand.
constexpr std::size_t probed_cases = 16;

// The two pairs the third case reads apart each return one type, so a comparator chosen by return
// type could not tell the members of a pair apart.
static_assert(std::is_same_v<matrix3, rotation>);
static_assert(std::is_same_v<matrix4, transform>);

std::size_t every_slot()
{
    return static_cast<std::size_t>(rigid_motion::frame_slot::count) + static_cast<std::size_t>(rigid_motion::screw_slot::count);
}

std::size_t agreeing(const evaluation_report &reported)
{
    return static_cast<std::size_t>(std::count_if(reported.slots.begin(), reported.slots.end(), [](const slot_report &row) { return row.verdict == agreement::agreed; }));
}

std::optional<residual_kind> kind_of(const std::array<evaluation_view, 2> &compared, std::string_view slot)
{
    for(const evaluation_view &view : compared)
        for(const slot_evaluation &row : view.slots())
            if(row.name == slot)
                return row.kind;

    return std::nullopt;
}

bool bend_shows_within(std::size_t index, const rigid_motion::capabilities &reference, const rigid_motion::capabilities &bent, std::size_t cases)
{
    case_source drawn = case_source::for_slot(default_seed, fixture::bent_slot_name(index));
    bool shown        = false;

    for(std::size_t at = 0; at < cases; ++at)
        shown = fixture::bend_is_visible(index, reference, bent, drawn) || shown;

    return shown;
}

}

TEST_CASE("every_bend_answers_differently_from_the_reference_at_the_inputs_the_run_draws")
{
    const rigid_motion::capabilities reference = rigid_motion::baseline();

    for(std::size_t index = 0; index < every_slot(); ++index)
    {
        const rigid_motion::capabilities bent = fixture::bent_at(index);

        INFO("slot " << index << ", " << fixture::bent_slot_name(index));
        REQUIRE_FALSE(fixture::bent_slot_name(index).empty());
        REQUIRE(bend_shows_within(index, reference, bent, probed_cases));
    }
}

TEST_CASE("bending_one_slot_makes_the_facility_name_that_slot_and_leave_every_other_one_agreeing")
{
    const rigid_motion::capabilities reference = rigid_motion::baseline();

    for(std::size_t index = 0; index < every_slot(); ++index)
    {
        const rigid_motion::capabilities bent         = fixture::bent_at(index);
        const std::array<evaluation_view, 2> compared = rigid_motion::evaluation_views(reference, bent);
        const evaluation_report reported              = evaluate(compared);
        const std::vector<std::string_view> named     = disagreeing_slots(reported);

        INFO("slot " << index << ", " << fixture::bent_slot_name(index));
        REQUIRE(reported.slots.size() == every_slot());
        REQUIRE(named.size() == 1u);
        REQUIRE(named.front() == fixture::bent_slot_name(index));
        REQUIRE(agreeing(reported) == every_slot() - 1u);
        REQUIRE_FALSE(every_slot_agreed(reported));
    }
}

TEST_CASE("an_index_past_the_last_slot_bends_nothing_and_names_nothing")
{
    const rigid_motion::capabilities reference    = rigid_motion::baseline();
    const rigid_motion::capabilities unbent       = fixture::bent_at(every_slot());
    const std::array<evaluation_view, 2> compared = rigid_motion::evaluation_views(reference, unbent);

    REQUIRE(fixture::bent_slot_name(every_slot()).empty());
    REQUIRE(every_slot_agreed(evaluate(compared, default_seed, probed_cases)));
}

TEST_CASE("a_comparator_belongs_to_the_slot_it_names_and_not_to_the_type_that_slot_returns")
{
    const rigid_motion::capabilities reference    = rigid_motion::baseline();
    const std::array<evaluation_view, 2> compared = rigid_motion::evaluation_views(reference, reference);

    REQUIRE(kind_of(compared, "screw.skew_symmetric") == residual_kind::element_wise);
    REQUIRE(kind_of(compared, "frame.rotate_x") == residual_kind::geodesic);
    REQUIRE(kind_of(compared, "screw.twist_matrix_from_twist") == residual_kind::element_wise);
    REQUIRE(kind_of(compared, "frame.inverse") == residual_kind::pose);
}
