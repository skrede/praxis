#include "book_inverse_kinematics.h"

#include "praxis/manipulator/evaluation.h"
#include "praxis/manipulator/kinematics.h"
#include "praxis/manipulator/capabilities.h"

#include "praxis/evaluation/report.h"
#include "praxis/evaluation/slot_evaluation.h"

#include <catch2/catch_test_macros.hpp>

#include <span>
#include <array>
#include <cstddef>
#include <cstdint>
#include <utility>
#include <algorithm>

using namespace praxis;
using namespace praxis::manipulator;
using namespace praxis::evaluation;
using namespace praxis::tests::book;

namespace {

constexpr std::array<std::uint64_t, 5> seeds{0x5EEDu, 0xC0FFEEu, 0xA11CEu, 0xBEEFu, 0xD1CEu};
constexpr std::size_t cases_per_seed = default_cases_per_slot;

// The form, refusing an answer at which its own error twist is not finite.
template<std::size_t I>
expected<void, refusal> refusing_a_non_finite_stop(const rigid_motion::screw_ops &screw, const forward_kinematics_ops &forward, const differential_kinematics_ops &differential,
                                                   const screw_chain &chain, const transform &desired, const joint_vector &j0, const solver_parameters &parameters, ik_result &answer)
{
    constexpr literal_form form          = literal_forms[I];
    const expected<void, refusal> solved = form.solve(screw, forward, differential, chain, desired, j0, parameters, answer);
    if(!solved || answer.solutions.empty())
        return solved;
    const expected<twist, refusal> own = error_twist<form.frame, form.source>(handed_maps{screw, forward, differential, chain}, desired, answer.solutions.front());
    if(!own || own->allFinite())
        return solved;
    answer.solutions.clear();
    return unexpected(refusal::no_solution);
}

template<std::size_t... I>
constexpr std::array<tests::book::inverse_kinematics_slot, sizeof...(I)> refusing_forms_of(std::index_sequence<I...>)
{
    return {&refusing_a_non_finite_stop<I>...};
}

constexpr std::array<tests::book::inverse_kinematics_slot, literal_forms.size()> refusing_forms = refusing_forms_of(std::make_index_sequence<literal_forms.size()>{});

bool carries_the_inverse_kinematics_row(const evaluation_view &view)
{
    return std::ranges::any_of(view.slots(), [](const slot_evaluation &slot) { return slot.name == "ik.inverse_kinematics"; });
}

slot_report judged(tests::book::inverse_kinematics_slot solve, std::uint64_t seed)
{
    const capabilities reference               = manipulator::baseline();
    capabilities literal                       = manipulator::baseline();
    literal.ik.inverse_kinematics              = solve;
    const std::array<evaluation_view, 7> views = evaluation_views(reference, literal);
    const auto row                             = std::ranges::find_if(views, &carries_the_inverse_kinematics_row);
    REQUIRE(row != views.end());

    return evaluate(std::span(row, 1), seed, cases_per_seed).slots.at(0);
}

}

TEST_CASE("every_literal_inverse_kinematics_form_on_the_companion_code_helpers_passes_the_inverse_kinematics_row")
{
    std::size_t forms_judged = 0;
    for(const literal_form &form : literal_forms)
    {
        if(form.source != helpers::handed)
            continue;
        for(const std::uint64_t seed : seeds)
        {
            const slot_report row = judged(form.solve, seed);

            INFO(form_label(form) << ", seed " << seed);
            REQUIRE(row.slot == "ik.inverse_kinematics");
            REQUIRE(row.cases == cases_per_seed);
            CHECK(row.outcomes.differed + row.outcomes.refused_differently + row.outcomes.unusable == 0u);
            CHECK(row.outcomes.agreed > 0u);
        }
        ++forms_judged;
    }

    CHECK(forms_judged == 12u);
}

// The same form refusing wherever its own error twist is not finite leaves every other case as it was.
TEST_CASE("a_literal_inverse_kinematics_form_on_the_text_logarithm_differs_from_the_row_only_where_its_own_error_twist_is_not_finite")
{
    std::size_t forms_judged = 0;
    for(std::size_t index = 0; index < literal_forms.size(); ++index)
    {
        if(literal_forms.at(index).source != helpers::text)
            continue;
        for(const std::uint64_t seed : seeds)
        {
            const slot_report row    = judged(literal_forms.at(index).solve, seed);
            const slot_report finite = judged(refusing_forms.at(index), seed);

            INFO(form_label(literal_forms.at(index)) << ", seed " << seed);
            REQUIRE(row.slot == "ik.inverse_kinematics");
            REQUIRE((row.cases == cases_per_seed && finite.cases == cases_per_seed));
            CHECK(row.outcomes.refused_differently + row.outcomes.unusable == 0u);
            CHECK(finite.outcomes.differed + finite.outcomes.refused_differently + finite.outcomes.unusable == 0u);
            CHECK(finite.outcomes.agreed > 0u);
        }
        ++forms_judged;
    }

    CHECK(forms_judged == 12u);
}
