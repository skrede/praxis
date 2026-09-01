#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_MODEL_TOPOLOGY_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_MODEL_TOPOLOGY_H

#include "praxis/extension/refusal.h"

#include "praxis/compat/expected.h"

#include <meios/model.h>

namespace praxis::manipulator {

// Holds when the model has a root, every entry of its parent, traversal-order and root tables
// addresses a link that exists, and every parent chain reaches a root in a bounded number of steps.
// The walks downstream index those tables and follow those chains without a bound of their own.
expected<void, refusal> addressable_topology(const meios::model<> &model);

}

#endif
