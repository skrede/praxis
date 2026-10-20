#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_EVALUATION_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_EVALUATION_H

#include "praxis/manipulator/capabilities.h"

#include "praxis/optimization/capabilities.h"

#include "praxis/evaluation/slot_evaluation.h"

#include <array>

namespace praxis::manipulator {

// The views are the capabilities a comparison is defined over, in the aggregate's member order, and
// point into both values passed, which must outlive them. A temporary argument would
// leave every view dangling at the end of the full expression, so those calls are deleted rather
// than diagnosed at run time.
std::array<evaluation::evaluation_view, 7> evaluation_views(const capabilities &first, const capabilities &second);
std::array<evaluation::evaluation_view, 7> evaluation_views(capabilities &&, const capabilities &) = delete;
std::array<evaluation::evaluation_view, 7> evaluation_views(const capabilities &, capabilities &&) = delete;
std::array<evaluation::evaluation_view, 7> evaluation_views(capabilities &&, capabilities &&)      = delete;

// A bound solver judged on the arm: each case drives both sides through the body or the space inverse
// kinematics adapter, drawn per case, and holds each answer, read back through the reference forward
// map, against the pose asked for. The view points into both values passed, which must outlive it, so
// a temporary argument is deleted as above.
std::array<evaluation::evaluation_view, 1> solver_evaluation_views(const optimization::capabilities &first, const optimization::capabilities &second);
std::array<evaluation::evaluation_view, 1> solver_evaluation_views(optimization::capabilities &&, const optimization::capabilities &) = delete;
std::array<evaluation::evaluation_view, 1> solver_evaluation_views(const optimization::capabilities &, optimization::capabilities &&) = delete;
std::array<evaluation::evaluation_view, 1> solver_evaluation_views(optimization::capabilities &&, optimization::capabilities &&)      = delete;

}

#endif
