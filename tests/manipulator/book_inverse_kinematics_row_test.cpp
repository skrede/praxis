#include "three_link_arm.h"
#include "book_inverse_kinematics.h"

#include "praxis/manipulator/evaluation.h"
#include "praxis/manipulator/kinematics.h"
#include "praxis/manipulator/capabilities.h"

#include "praxis/evaluation/report.h"
#include "praxis/evaluation/tolerance.h"
#include "praxis/evaluation/slot_evaluation.h"

#include "praxis/rigid_motion/capabilities.h"

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
expected<void, refusal> refusing_a_non_finite_stop(const rigid_motion::screw_ops &screw, const rigid_motion::frame_ops &frames, const forward_kinematics_ops &forward,
                                                   const differential_kinematics_ops &differential, const screw_chain &chain, const transform &desired, const joint_vector &j0,
                                                   const solver_parameters &parameters, ik_result &answer)
{
    constexpr literal_form form          = literal_forms[I];
    const expected<void, refusal> solved = form.solve(screw, frames, forward, differential, chain, desired, j0, parameters, answer);
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

// [V_b] = log(T_sb^-1 T_sd), with T_sb and its inverse asked of the operations the solve is handed.
expected<twist, refusal> body_error(const rigid_motion::screw_ops &screw, const rigid_motion::frame_ops &frames, const forward_kinematics_ops &forward, const screw_chain &chain,
                                    const transform &desired, const joint_vector &theta)
{
    const expected<transform, refusal> tsb = forward.body_forward_kinematics(screw, frames, chain.home, chain.space_screws, theta);
    if(!tsb)
        return unexpected(tsb.error());
    const expected<pose_reading, refusal> logarithm = screw.matrix_logarithm_se3(frames.inverse(*tsb) * desired);
    if(!logarithm)
        return unexpected(logarithm.error());
    return twist(logarithm->first * logarithm->second);
}

// Lynch & Park, Modern Robotics, sec. 6.2.2, the body-frame iteration with the step of eq. (6.6) and
// J_b asked of the differential maps it is handed.
expected<void, refusal> body_frame_literal(const rigid_motion::screw_ops &screw, const rigid_motion::frame_ops &frames, const forward_kinematics_ops &forward,
                                           const differential_kinematics_ops &differential, const screw_chain &chain, const transform &desired, const joint_vector &j0,
                                           const solver_parameters &parameters, ik_result &answer)
{
    joint_vector theta         = j0;
    expected<twist, refusal> v = body_error(screw, frames, forward, chain, desired, theta);
    for(std::uint32_t i = 0; v && outside(*v, parameters) && i < parameters.max_iterations_per_attempt; ++i)
    {
        const expected<jacobian, refusal> jb = differential.body_jacobian(screw, frames, forward, chain.home, chain.space_screws, theta);
        if(!jb)
            return unexpected(jb.error());
        theta += step_along<step::cod>(*jb, *v);
        v = body_error(screw, frames, forward, chain, desired, theta);
    }
    if(!v || outside(*v, parameters))
        return unexpected(refusal::no_solution);
    answer.solutions.push_back(theta);
    return {};
}

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

TEST_CASE("a_literal_body_frame_solve_reading_its_maps_through_the_frames_it_is_handed_passes_the_inverse_kinematics_row")
{
    for(std::uint64_t seed = default_seed; seed <= default_seed + 40u; ++seed)
    {
        const slot_report row = judged(&body_frame_literal, seed);

        INFO("seed " << seed);
        REQUIRE(row.slot == "ik.inverse_kinematics");
        REQUIRE(row.cases == cases_per_seed);
        CHECK(row.outcomes.differed + row.outcomes.refused_differently + row.outcomes.unusable == 0u);
        CHECK(row.outcomes.agreed > 0u);
    }
}

TEST_CASE("the_literal_body_frame_solve_handed_default_frame_operations_does_not_reach_a_target_the_reference_frames_reach")
{
    const screw_chain chain                   = fixture::three_link_arm();
    const rigid_motion::capabilities spatial  = rigid_motion::baseline();
    const capabilities maps                   = manipulator::baseline();
    const expected<transform, refusal> target = maps.fk.forward_kinematics(spatial.screw, chain.home, chain.space_screws, fixture::posed_arm());
    const joint_vector seed                   = fixture::arm_configuration(0.3, -1.0, 0.4);
    REQUIRE(target);

    ik_result composed;
    ik_result defaulted;
    const expected<void, refusal> reached = body_frame_literal(spatial.screw, spatial.frame, maps.fk, maps.dk, chain, *target, seed, solver_parameters(), composed);
    const expected<void, refusal> missed  = body_frame_literal(spatial.screw, rigid_motion::frame_ops{}, maps.fk, maps.dk, chain, *target, seed, solver_parameters(), defaulted);

    REQUIRE(reached);
    REQUIRE(composed.solutions.size() == 1u);
    const expected<transform, refusal> landed = maps.fk.forward_kinematics(spatial.screw, chain.home, chain.space_screws, composed.solutions.front());
    REQUIRE(landed);
    CHECK(is_approx_equal(*landed, *target, fixture::solved_tolerance));
    REQUIRE_FALSE(missed);
    CHECK(missed.error() == refusal::no_solution);
    CHECK(defaulted.solutions.empty());
}
