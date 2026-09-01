#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_MODELING_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_MODELING_H

#include "praxis/manipulator/screw_chain.h"

#include "praxis/extension/refusal.h"

#include "praxis/compat/expected.h"

#include <meios/model.h>

namespace praxis::manipulator::inert {

expected<screw_chain, refusal> build_chain(const meios::model<> &model);

}

namespace praxis::manipulator {

// Declaration order is frozen: a designated initializer must name members in declaration order, so
// reordering a slot breaks every project that already composes this aggregate. Appending is safe.
struct modeling_ops
{
    expected<screw_chain, refusal> (*build_chain)(const meios::model<> &model) = &inert::build_chain;
};

}

#endif
