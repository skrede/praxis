#ifndef HPP_GUARD_PRAXIS_OPTIMIZATION_SLOTS_H
#define HPP_GUARD_PRAXIS_OPTIMIZATION_SLOTS_H

#include "praxis/optimization/solver.h"

#include "praxis/extension/coverage.h"
#include "praxis/extension/slot_set.h"
#include "praxis/extension/descriptor.h"

#include <cstdint>

namespace praxis::optimization {

// The enumerators are the aggregate's member names unqualified and in its declaration order: the
// capability is carried by the enumeration's own name, so a slot another extension spells the same
// way stays distinct. The trailing count enumerator names no slot; the slot set, the size assertions
// and the coverage functions all read it.
enum class solver_slot : std::uint32_t
{
    solve,
    count,
};

using solver_slot_set = basic_slot_set<solver_slot>;

// A view points into the value it was given, which must outlive it. A temporary argument would leave
// the view dangling at the end of the full expression, so that call is deleted rather than diagnosed
// at run time.
capability_view view_of(const solver_ops &ops);
capability_view view_of(solver_ops &&) = delete;

}

// The enumeration the aggregate's slots are indexed by, so a set of one capability's slots cannot be
// read against another's descriptor table.
namespace praxis {

template<>
struct capability_slots<optimization::solver_ops>
{
    using type = optimization::solver_slot;
};

}

#endif
