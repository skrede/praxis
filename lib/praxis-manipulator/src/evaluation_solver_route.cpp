#include "evaluation_cases.h"
#include "evaluation_tables.h"

#include "praxis/manipulator/kinematics.h"
#include "praxis/manipulator/solver_adapters.h"

#include "praxis/optimization/slots.h"
#include "praxis/optimization/solver.h"

#include "praxis/evaluation/comparators.h"

#include <array>
#include <cstddef>
#include <optional>

namespace praxis::manipulator {

namespace {

using adapter = decltype(inverse_kinematics_ops::inverse_kinematics);

const optimization::solver_ops &solver_of(const void *value)
{
    return *static_cast<const optimization::solver_ops *>(value);
}

expected<void, refusal> driven_through(adapter route, const optimization::solver_ops &solver, const screw_chain &chain, const solve_request &asked, ik_result &answer)
{
    const expected<void, refusal> solved =
            route(shared_screw(), shared_frames(), asked.forward, asked.differential, solver, chain, asked.target, asked.seed, solver_parameters(), answer);

    return answered_with_a_configuration(solved, answer);
}

// The row is in the enumerator order of solver_slot and spelled as the descriptor table spells it.
constexpr std::array solver_route_table{
        evaluation::slot_evaluation{"solver.solve", evaluation::residual_kind::pose, evaluation::tolerance_pair{solver_route_tolerance_radians, solver_route_tolerance_meters},
                                    &compare_solver_route},
};

static_assert(solver_route_table.size() == static_cast<std::size_t>(optimization::solver_slot::count));

constexpr evaluation::capability_evaluations<optimization::solver_ops> evaluated_solver_route{"manipulator", solver_route_table};

}

// Both sides take the same adapter on a case, the body form below one half of the draw and the space
// form above it.
evaluation::case_result compare_solver_route(const void *first, const void *second, evaluation::case_source &drawn, const evaluation::tolerance_pair &allowed)
{
    const evaluation_case example            = drawn_case(drawn);
    const std::optional<solve_request> asked = drawn_request(drawn, example);
    if(!asked)
        return unusable(evaluation::residual_kind::pose);

    const adapter route = drawn.uniform(0.0, 1.0) < 0.5 ? &body_inverse_kinematics : &space_inverse_kinematics;
    ik_result here;
    ik_result there;
    const expected<void, refusal> held  = driven_through(route, solver_of(first), example.chain, *asked, here);
    const expected<void, refusal> other = driven_through(route, solver_of(second), example.chain, *asked, there);
    if(const std::optional<evaluation::case_result> refused = refusal_outcome(held, other))
        return *refused;

    return reaching_what_was_asked(reached_by(asked->reference, here), reached_by(asked->reference, there), asked->target, allowed);
}

const evaluation::capability_evaluations<optimization::solver_ops> &solver_route_evaluations()
{
    return evaluated_solver_route;
}

}
