#include "bent_chain.h"
#include "evaluation_cases.h"
#include "evaluation_tables.h"

#include "praxis/manipulator/capabilities.h"
#include "praxis/manipulator/baseline/modeling.h"
#include "praxis/manipulator/baseline/kinematics.h"

#include "praxis/rigid_motion/capabilities.h"

#include "praxis/evaluation/residual.h"
#include "praxis/evaluation/generation.h"
#include "praxis/evaluation/comparators.h"
#include "praxis/evaluation/slot_evaluation.h"

#include <catch2/catch_test_macros.hpp>

#include <meios/model.h>

#include <cmath>
#include <vector>
#include <cstddef>
#include <cstdint>
#include <string_view>

using namespace praxis;
using namespace praxis::fixture;
using namespace praxis::evaluation;

namespace {

constexpr std::uint64_t recorded_seed = 0xC0FFEEu;
constexpr std::size_t cases_per_row   = 8u;

using comparator = case_result (*)(const void *, const void *, case_source &, const tolerance_pair &);

// Each row is judged at the bound its own shipped table carries, which for the solve row is not the kind's default.
constexpr tolerance_pair derived_bound = tolerance_of(residual_kind::element_wise);
constexpr tolerance_pair built_bound   = tolerance_of(residual_kind::pose);
constexpr tolerance_pair solved_bound{manipulator::solved_pose_tolerance_radians, manipulator::solved_pose_tolerance_metres};

case_result one_case(comparator compare, std::string_view slot, const void *first, const void *second, const tolerance_pair &allowed, std::size_t index)
{
    case_source drawn = case_source::at_case(recorded_seed, slot, spread::bulk, index);

    return compare(first, second, drawn, allowed);
}

}

TEST_CASE("the_reference_agrees_with_itself_on_the_three_rows_the_shipped_vocabulary_could_not_serve")
{
    const capabilities reference = baseline();

    for(std::size_t index = 0; index < cases_per_row; ++index)
    {
        INFO("case " << index);
        const case_result derived = one_case(&manipulator::compare_body_screws_from_space, "fk.body_screws_from_space", &reference.fk, &reference.fk, derived_bound, index);
        const case_result solved  = one_case(&manipulator::compare_inverse_kinematics, "ik.inverse_kinematics", &reference.ik, &reference.ik, solved_bound, index);
        const case_result built   = one_case(&manipulator::compare_build_chain, "modeling.build_chain", &reference.modeling, &reference.modeling, built_bound, index);

        REQUIRE(derived.verdict == agreement::agreed);
        REQUIRE((solved.verdict == agreement::agreed || solved.verdict == agreement::both_refused));
        REQUIRE(built.verdict == agreement::agreed);
    }
}

TEST_CASE("a_derivation_that_negates_every_axis_differs_here_and_would_not_be_seen_up_to_sign")
{
    const capabilities reference   = baseline();
    forward_kinematics_ops flipped = reference.fk;
    flipped.body_screws_from_space = &every_axis_negated;

    for(std::size_t index = 0; index < cases_per_row; ++index)
    {
        INFO("case " << index);
        const case_result seen = one_case(&manipulator::compare_body_screws_from_space, "fk.body_screws_from_space", &reference.fk, &flipped, derived_bound, index);

        REQUIRE(seen.verdict == agreement::differed);
        REQUIRE(seen.difference.magnitude > element_wise_tolerance);

        case_source drawn             = case_source::at_case(recorded_seed, "fk.body_screws_from_space", spread::bulk, index);
        const evaluation_case example = drawn_case(drawn);
        const auto derived            = body_screws_from_space(rigid_motion::baseline().screw, rigid_motion::baseline().frame, example.chain.home, example.chain.space_screws).value();
        for(const screw_axis &axis : derived)
            REQUIRE(axis_up_to_sign_residual(axis, screw_axis(-axis)).magnitude <= axis_up_to_sign_tolerance);
    }
}

TEST_CASE("two_derivations_of_different_length_differ_whatever_their_common_prefix_holds")
{
    const capabilities reference     = baseline();
    forward_kinematics_ops shortened = reference.fk;
    shortened.body_screws_from_space = &one_axis_short;

    for(std::size_t index = 0; index < cases_per_row; ++index)
    {
        INFO("case " << index);
        const case_result seen = one_case(&manipulator::compare_body_screws_from_space, "fk.body_screws_from_space", &reference.fk, &shortened, derived_bound, index);

        REQUIRE(seen.verdict == agreement::differed);
        REQUIRE(std::isinf(seen.difference.magnitude));
    }
}

TEST_CASE("a_solve_that_declines_is_reported_apart_from_one_that_answers_and_from_one_that_declines_alike")
{
    const inverse_kinematics_ops declines  = solving(&never_solves);
    const inverse_kinematics_ops otherwise = solving(&never_solves_for_another_reason);
    const inverse_kinematics_ops answers   = solving(&always_answers_the_seed);

    for(std::size_t index = 0; index < cases_per_row; ++index)
    {
        INFO("case " << index);
        REQUIRE(one_case(&manipulator::compare_inverse_kinematics, "ik.inverse_kinematics", &answers, &declines, solved_bound, index).verdict == agreement::one_refused);
        REQUIRE(one_case(&manipulator::compare_inverse_kinematics, "ik.inverse_kinematics", &declines, &declines, solved_bound, index).verdict == agreement::both_refused);
        REQUIRE(one_case(&manipulator::compare_inverse_kinematics, "ik.inverse_kinematics", &declines, &otherwise, solved_bound, index).verdict == agreement::refused_differently);
    }
}

TEST_CASE("a_solve_that_answers_without_naming_a_configuration_differs_rather_than_refusing")
{
    const capabilities reference          = baseline();
    const inverse_kinematics_ops silently = solving(&answers_without_naming_a_configuration);

    for(std::size_t index = 0; index < cases_per_row; ++index)
    {
        INFO("case " << index);
        const case_result seen = one_case(&manipulator::compare_inverse_kinematics, "ik.inverse_kinematics", &reference.ik, &silently, solved_bound, index);

        REQUIRE((seen.verdict == agreement::differed || seen.verdict == agreement::one_refused));
    }
    REQUIRE(one_case(&manipulator::compare_inverse_kinematics, "ik.inverse_kinematics", &silently, &silently, solved_bound, 0u).verdict == agreement::differed);
}

TEST_CASE("the_comparison_enters_the_solve_under_test_exactly_once_per_case")
{
    const capabilities reference          = baseline();
    const inverse_kinematics_ops counting = solving(&counts_its_entries);

    entries = 0;
    for(std::size_t index = 0; index < cases_per_row; ++index)
        one_case(&manipulator::compare_inverse_kinematics, "ik.inverse_kinematics", &counting, &reference.ik, solved_bound, index);

    REQUIRE(entries == cases_per_row);
}

TEST_CASE("a_chain_with_one_screw_negated_differs")
{
    const capabilities reference = baseline();
    modeling_ops flipped         = reference.modeling;
    flipped.build_chain          = &last_screw_negated;

    for(std::size_t index = 0; index < cases_per_row; ++index)
    {
        INFO("case " << index);
        const case_result seen = one_case(&manipulator::compare_build_chain, "modeling.build_chain", &reference.modeling, &flipped, built_bound, index);

        REQUIRE(seen.verdict == agreement::differed);
    }
}

TEST_CASE("a_chain_of_a_different_joint_count_differs_rather_than_refusing")
{
    const capabilities reference = baseline();
    modeling_ops shortened       = reference.modeling;
    shortened.build_chain        = &one_joint_short;

    for(std::size_t index = 0; index < cases_per_row; ++index)
    {
        INFO("case " << index);
        const case_result seen = one_case(&manipulator::compare_build_chain, "modeling.build_chain", &reference.modeling, &shortened, built_bound, index);

        REQUIRE(seen.verdict == agreement::differed);
        REQUIRE(std::isinf(seen.difference.magnitude));
    }
}

TEST_CASE("a_chain_agreeing_at_one_drawn_configuration_and_not_the_others_still_differs")
{
    const capabilities reference = baseline();
    modeling_ops partly          = reference.modeling;
    partly.build_chain           = &agreeing_at_one_configuration;

    for(std::size_t index = 0; index < cases_per_row; ++index)
    {
        INFO("case " << index);
        case_source drawn            = case_source::at_case(recorded_seed, "modeling.build_chain", spread::bulk, index);
        const meios::model<> machine = drawn_model(drawn);
        agreeing_configuration       = drawn_joints(drawn, machine.joints.size());

        const screw_chain right     = build_chain(machine).value();
        const screw_chain wrong     = agreeing_at_one_configuration(machine).value();
        const transform here        = forward_kinematics(right.home, right.space_screws, agreeing_configuration).value();
        const transform there       = forward_kinematics(wrong.home, wrong.space_screws, agreeing_configuration).value();
        const residual at_the_first = pose_residual(here, there);

        REQUIRE(at_the_first.magnitude <= pose_tolerance_radians);
        REQUIRE(at_the_first.linear_error_metres <= pose_tolerance_metres);
        REQUIRE(one_case(&manipulator::compare_build_chain, "modeling.build_chain", &reference.modeling, &partly, built_bound, index).verdict == agreement::differed);
    }
}
