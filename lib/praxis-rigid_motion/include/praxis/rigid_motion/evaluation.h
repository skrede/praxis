#ifndef HPP_GUARD_PRAXIS_RIGID_MOTION_EVALUATION_H
#define HPP_GUARD_PRAXIS_RIGID_MOTION_EVALUATION_H

#include "praxis/rigid_motion/capabilities.h"

#include "praxis/evaluation/slot_evaluation.h"

#include <array>

namespace praxis::rigid_motion {

// The views are in the aggregate's member order and point into both values passed, which must
// outlive them. A temporary argument would leave every view dangling at the end of the full
// expression, so those calls are deleted rather than diagnosed at run time.
std::array<evaluation::evaluation_view, 2> evaluation_views(const capabilities &first, const capabilities &second);
std::array<evaluation::evaluation_view, 2> evaluation_views(capabilities &&, const capabilities &) = delete;
std::array<evaluation::evaluation_view, 2> evaluation_views(const capabilities &, capabilities &&) = delete;
std::array<evaluation::evaluation_view, 2> evaluation_views(capabilities &&, capabilities &&)      = delete;

}

#endif
