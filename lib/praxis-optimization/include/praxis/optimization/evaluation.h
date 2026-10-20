#ifndef HPP_GUARD_PRAXIS_OPTIMIZATION_EVALUATION_H
#define HPP_GUARD_PRAXIS_OPTIMIZATION_EVALUATION_H

#include "praxis/optimization/capabilities.h"

#include "praxis/evaluation/slot_evaluation.h"

#include <array>

namespace praxis::optimization {

// A solve is judged by its answer and never by the iterates it reached it through: where both sides
// answer, a case agrees exactly when the first side's answer satisfies the problem's own convergence
// test. The views are in the aggregate's member order and point into both values passed, which must
// outlive them. A temporary argument would leave every view dangling at the end of the full
// expression, so those calls are deleted rather than diagnosed at run time.
std::array<evaluation::evaluation_view, 1> evaluation_views(const capabilities &first, const capabilities &second);
std::array<evaluation::evaluation_view, 1> evaluation_views(capabilities &&, const capabilities &) = delete;
std::array<evaluation::evaluation_view, 1> evaluation_views(const capabilities &, capabilities &&) = delete;
std::array<evaluation::evaluation_view, 1> evaluation_views(capabilities &&, capabilities &&)      = delete;

}

#endif
