#ifndef HPP_GUARD_PRAXIS_OPTIMIZATION_CAPABILITIES_H
#define HPP_GUARD_PRAXIS_OPTIMIZATION_CAPABILITIES_H

#include "praxis/optimization/slots.h"

#include "praxis/extension/descriptor.h"

#include <array>

namespace praxis::optimization {

struct capabilities
{
    solver_ops solver{};
    minimizer_ops minimizer{};
};

capabilities baseline();

// The views are in the aggregate's member order and point into the value passed, which must outlive
// them. A temporary argument would leave every view dangling at the end of the full expression, so
// that call is deleted rather than diagnosed at run time.
std::array<capability_view, 2> capability_views(const capabilities &c);
std::array<capability_view, 2> capability_views(capabilities &&) = delete;

}

#endif
