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

const evaluation::capability_evaluations<solver_ops> &solver_evaluations();

}

#endif
