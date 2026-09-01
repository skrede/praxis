#include "bent_manipulator.h"
#include "bend_probe.h"

#include "praxis/manipulator/evaluation.h"
#include "praxis/manipulator/capabilities.h"

#include "praxis/evaluation/slot_evaluation.h"

#include <catch2/catch_test_macros.hpp>

#include <array>
#include <vector>
#include <cstddef>
#include <string_view>

using namespace praxis;
using namespace praxis::evaluation;

namespace {

// Far above every bound the tables carry, so this file asks whether a bend reaches the answer at all
// and never whether it clears a tolerance.
constexpr std::array<double, static_cast<std::size_t>(fixture::bent::count)> plainly_wrong{1.0e-3, 1.0e-3, 1.0e-3, 1.0e-3, 1.0e-3, 1.0e-3};

std::vector<std::string_view> named_in_order(const manipulator::capabilities &arm)
{
    std::vector<std::string_view> named;

    for(const evaluation_view &view : manipulator::evaluation_views(arm, arm))
        for(const slot_evaluation &slot : view.slots())
            named.push_back(slot.name);

    return named;
}

using fixture::every_row;
using fixture::apart_over_the_run;

}

TEST_CASE("every_bend_answers_differently_from_the_reference_at_the_inputs_the_run_draws")
{
    const manipulator::capabilities reference = manipulator::baseline();
    const std::vector<std::string_view> named = named_in_order(reference);

    fixture::bend_every_row_by(plainly_wrong);

    const std::array<bool, every_row> seen = apart_over_the_run(reference, fixture::bent_everywhere());

    fixture::bend_every_row_by({});

    for(std::size_t row = 0; row < every_row; ++row)
    {
        INFO("row " << row << ", " << named[row]);
        REQUIRE(seen[row]);
    }
}

TEST_CASE("a_bend_of_one_kind_moves_the_rows_that_kind_names_and_leaves_every_other_row_at_the_reference")
{
    const manipulator::capabilities reference = manipulator::baseline();
    const std::vector<std::string_view> named = named_in_order(reference);

    for(std::size_t which = 0; which < static_cast<std::size_t>(fixture::bent::count); ++which)
    {
        std::array<double, static_cast<std::size_t>(fixture::bent::count)> one{};
        one[which] = 1.0e-3;

        fixture::bend_every_row_by(one);

        const std::array<bool, every_row> seen = apart_over_the_run(reference, fixture::bent_everywhere());

        fixture::bend_every_row_by({});

        for(std::size_t row = 0; row < every_row; ++row)
        {
            INFO("bend " << which << ", row " << named[row]);
            REQUIRE(seen[row] == fixture::is_bent(static_cast<fixture::bent>(which), named[row]));
        }
    }
}
