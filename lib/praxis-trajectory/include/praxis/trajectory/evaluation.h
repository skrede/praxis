#ifndef HPP_GUARD_PRAXIS_TRAJECTORY_EVALUATION_H
#define HPP_GUARD_PRAXIS_TRAJECTORY_EVALUATION_H

#include "praxis/trajectory/trajectory.h"
#include "praxis/trajectory/capabilities.h"
#include "praxis/trajectory/pose_trajectory.h"

#include "praxis/evaluation/residual.h"
#include "praxis/evaluation/slot_evaluation.h"

#include <array>

namespace praxis::trajectory {

// Two prepared motions compared by what they compute: their durations first, then their samples at a
// fixed set of times over the span both answer. Two spans that differ share no interval to sample
// over and are a difference on their own. A time either side declines is carried through the
// facility's refusal policy rather than turned into a number. Published here because the generator a
// factory answers is this extension's own type, whichever extension's slot the factory fills.
evaluation::case_result driven(const trajectory_generator &held, const trajectory_generator &against, evaluation::residual_kind kind, const evaluation::tolerance_pair &allowed);
evaluation::case_result driven(const pose_trajectory_generator &held, const pose_trajectory_generator &against, evaluation::residual_kind kind,
                               const evaluation::tolerance_pair &allowed);

// The views are the capabilities a comparison is defined over, in the aggregate's member order, and
// point into both values passed, which must outlive them. A temporary argument would leave every
// view dangling at the end of the full expression, so those calls are deleted rather than diagnosed
// at run time.
std::array<evaluation::evaluation_view, 4> evaluation_views(const capabilities &first, const capabilities &second);
std::array<evaluation::evaluation_view, 4> evaluation_views(capabilities &&, const capabilities &) = delete;
std::array<evaluation::evaluation_view, 4> evaluation_views(const capabilities &, capabilities &&) = delete;
std::array<evaluation::evaluation_view, 4> evaluation_views(capabilities &&, capabilities &&)      = delete;

}

#endif
