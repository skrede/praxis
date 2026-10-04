#include "book_chain_maps.h"

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

constexpr std::array<std::uint64_t, 5> seeds{0x5EEDu, 0xC0FFEEu, 0xA11CEu, 0xBEEFu, 0xD1CEu};
constexpr std::size_t cases_per_seed = 10000u;

constexpr std::array<std::string_view, 6> chain_rows{"fk.forward_kinematics", "fk.body_forward_kinematics", "fk.body_screws_from_space",
                                                     "dk.space_jacobian",     "dk.body_jacobian",           "modeling.build_chain"};

bool is_a_chain_row(std::string_view slot)
{
    return std::ranges::find(chain_rows, slot) != chain_rows.end();
}

// Only the views carrying a chain row are evaluated: the others read the solve, which this file does
// not bind.
std::vector<evaluation_view> chain_views(const std::array<evaluation_view, 7> &every)
{
    std::vector<evaluation_view> picked;
    for(const evaluation_view &view : every)
        if(std::ranges::any_of(view.slots(), [](const slot_evaluation &slot) { return is_a_chain_row(slot.name); }))
            picked.push_back(view);

    return picked;
}

// Every outcome but agreement and a refusal both sides share.
std::size_t apart(const slot_report &row)
{
    return row.outcomes.differed + row.outcomes.one_refused + row.outcomes.refused_differently + row.outcomes.unusable;
}

}

TEST_CASE("a_literal_transcription_of_the_forward_maps_and_jacobians_agrees_on_every_case_at_five_seeds")
{
    const manipulator::capabilities reference   = manipulator::baseline();
    const manipulator::capabilities book        = tests::book::chain_maps();
    const std::vector<evaluation_view> compared = chain_views(manipulator::evaluation_views(reference, book));
    std::size_t rows_seen                       = 0;

    for(const std::uint64_t seed : seeds)
        for(const slot_report &row : evaluate(compared, seed, cases_per_seed).slots)
        {
            INFO("seed " << seed << ", " << row.slot);
            REQUIRE(is_a_chain_row(row.slot));
            REQUIRE(row.cases == cases_per_seed);
            REQUIRE(apart(row) == 0u);
            ++rows_seen;
        }

    REQUIRE(rows_seen == seeds.size() * chain_rows.size());
}
