#include "solve_rows.h"
#include "bent_chain.h"
#include "bent_solver.h"
#include "bend_kinds.h"
#include "bent_manipulator.h"
#include "evaluation_tables.h"

#include "praxis/manipulator/evaluation.h"
#include "praxis/manipulator/capabilities.h"

#include "praxis/evaluation/residual.h"
#include "praxis/evaluation/generation.h"
#include "praxis/evaluation/slot_evaluation.h"

#include <catch2/catch_test_macros.hpp>

#include <span>
#include <array>
#include <cmath>
#include <vector>
#include <cstddef>
#include <string_view>

using namespace praxis;
using namespace praxis::fixture;
using namespace praxis::manipulator;
using namespace praxis::evaluation;

namespace {

constexpr std::size_t swept_cases = 32u;

// Far above every bound the tables carry, so the pass that carries it asks whether a row reports a
// difference at all and never whether the difference clears a tolerance.
constexpr std::array<double, static_cast<std::size_t>(bent::count)> plainly_wrong{1.0e-3, 1.0e-3, 1.0e-3, 1.0e-3, 1.0e-3, 1.0e-3};

capabilities with_the_chain_bound_to(decltype(modeling_ops::build_chain) building)
{
    capabilities arm         = baseline();
    arm.modeling.build_chain = building;

    return arm;
}

capabilities with_the_solve_bound_to(decltype(inverse_kinematics_ops::inverse_kinematics) solving)
{
    capabilities arm               = baseline();
    arm.ik.inverse_kinematics      = solving;
    arm.robot.ik_solve_pose        = &solve_pose_for_a_displaced_target;
    arm.robot.ik_solve_flange_pose = &solve_flange_pose_for_a_displaced_target;

    return arm;
}

capabilities with_the_derivation_shortened()
{
    capabilities arm              = baseline();
    arm.fk.body_screws_from_space = &one_axis_short;

    return arm;
}

// Every case of every shipped row against one other aggregate, so a site reporting a difference it
// never measured is caught wherever it sits.
std::size_t differences_carrying_no_number(const capabilities &reference, const capabilities &other)
{
    std::size_t empty = 0;

    for(const evaluation_view &view : evaluation_views(reference, other))
        for(const slot_evaluation &row : view.slots())
            for(std::size_t index = 0; index < swept_cases; ++index)
            {
                case_source drawn      = case_source::at_case(recorded_seed, row.name, spread::bulk, index);
                const case_result seen = row.compare(view.first(), view.second(), drawn, row.allowed);
                if(seen.verdict != agreement::differed)
                    continue;

                empty += seen.difference.magnitude + seen.difference.linear_error_metres > 0.0 ? 0u : 1u;
            }

    return empty;
}

// A row's first case at which the reference answers, taken against the other aggregate. A case the
// reference declines is settled by the refusal policy standing in front of every site below.
case_result the_first_answered_case_of(std::string_view name, std::span<const slot_evaluation> table, const void *reference_side, const void *other)
{
    const slot_evaluation &row = named(table, name);
    for(std::size_t index = 0; index < swept_cases; ++index)
        if(one_case(row, reference_side, reference_side, index).verdict == agreement::agreed)
            return one_case(row, reference_side, other, index);

    return case_result{agreement::not_exercised, residual{}};
}

}

TEST_CASE("no_manipulator_row_reports_a_difference_with_nothing_in_it_at_any_case_of_any_bend")
{
    const capabilities reference = baseline();

    off_target_radians = 1.0e-3;
    off_target_metres  = 1.0e-3;
    bend_every_row_by(plainly_wrong);

    const std::array<capabilities, 7> swept{reference,
                                            with_the_derivation_shortened(),
                                            with_the_chain_bound_to(&one_joint_short),
                                            with_the_chain_bound_to(&a_home_that_is_no_rigid_motion),
                                            with_the_solve_bound_to(&answers_without_naming_a_configuration),
                                            with_the_solve_bound_to(&solves_for_a_displaced_target),
                                            bent_everywhere()};

    std::vector<std::size_t> empty;
    for(const capabilities &other : swept)
        empty.push_back(differences_carrying_no_number(reference, other));

    bend_every_row_by({});
    off_target_radians = 0.0;
    off_target_metres  = 0.0;

    for(std::size_t pass = 0; pass < empty.size(); ++pass)
    {
        INFO("pass " << pass);
        REQUIRE(empty[pass] == 0u);
    }
}

// The sweep above is only worth its name where it reaches the four sites whose difference no residual
// measures, which is what each row below stands for.
TEST_CASE("each_site_whose_difference_no_residual_measures_is_reached_and_answers_beyond_every_bound")
{
    const capabilities reference     = baseline();
    const capabilities shortened     = with_the_derivation_shortened();
    const capabilities narrower      = with_the_chain_bound_to(&one_joint_short);
    const capabilities unmappable    = with_the_chain_bound_to(&a_home_that_is_no_rigid_motion);
    const capabilities naming_no_one = with_the_solve_bound_to(&answers_without_naming_a_configuration);

    const case_result derived = the_first_answered_case_of("fk.body_screws_from_space", forward_kinematics_evaluations().slots, &reference.fk, &shortened.fk);
    const case_result built   = the_first_answered_case_of("modeling.build_chain", modeling_evaluations().slots, &reference.modeling, &narrower.modeling);
    const case_result mapped  = the_first_answered_case_of("modeling.build_chain", modeling_evaluations().slots, &reference.modeling, &unmappable.modeling);
    const case_result solved  = the_first_answered_case_of("ik.inverse_kinematics", inverse_kinematics_evaluations().slots, &reference.ik, &naming_no_one.ik);

    REQUIRE(derived.verdict == agreement::differed);
    REQUIRE(std::isinf(derived.difference.magnitude));
    REQUIRE(built.verdict == agreement::differed);
    REQUIRE(std::isinf(built.difference.magnitude));
    REQUIRE(mapped.verdict == agreement::unusable);
    REQUIRE(solved.verdict == agreement::differed);
    REQUIRE(std::isinf(solved.difference.magnitude));
}
