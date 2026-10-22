#ifndef HPP_GUARD_PRAXIS_OPTIMIZATION_EVALUATION_TABLES_H
#define HPP_GUARD_PRAXIS_OPTIMIZATION_EVALUATION_TABLES_H

#include "praxis/optimization/slots.h"

#include "praxis/evaluation/slot_evaluation.h"

#include <cstdint>

namespace praxis::optimization {

// The greatest element of a problem's error at which the solver row takes it as solved, in the
// problem's own units.
inline constexpr double solver_row_tolerance = 1.0e-12;

// The steps the solver row hands either side, a count.
inline constexpr std::uint32_t solver_row_iteration_limit = 70;

// The half-width of the box every coordinate of a seed is drawn from, in the problem's own
// coordinate, one per problem the solver row poses.
inline constexpr double polynomial_seed_half_width      = 4.0;
inline constexpr double transcendental_seed_half_width  = 1.0;
inline constexpr double square_system_seed_half_width   = 4.0;
inline constexpr double underdetermined_seed_half_width = 4.0;

// The greatest element of a gradient at which either minimizer row takes an answer as stationary, in
// the objective's units per unit of x.
inline constexpr double minimizer_row_tolerance = 1.0e-12;

// The steps each minimizer row hands either side, a count.
inline constexpr std::uint32_t minimizer_row_newton_limit  = 20;
inline constexpr std::uint32_t minimizer_row_descent_limit = 200;

// The half-width of the box every coordinate of a seed is drawn from, in the problem's own
// coordinate, one per problem the minimizer rows pose.
inline constexpr double quartic_seed_half_width          = 4.0;
inline constexpr double exponential_seed_half_width      = 2.0;
inline constexpr double elongated_bowl_seed_half_width   = 4.0;
inline constexpr double rosenbrock_seed_half_width       = 4.0;
inline constexpr double shifted_parabola_seed_half_width = 4.0;
inline constexpr double oval_bowl_seed_half_width        = 4.0;
inline constexpr double double_well_seed_half_width      = 2.0;

// The step length the descent row hands either side, one per problem it poses, in units of x per
// unit of gradient.
inline constexpr double shifted_parabola_step_length = 0.5;
inline constexpr double oval_bowl_step_length        = 0.2;
inline constexpr double double_well_step_length      = 0.5;

const evaluation::capability_evaluations<solver_ops> &solver_evaluations();
const evaluation::capability_evaluations<minimizer_ops> &minimizer_evaluations();

}

#endif
