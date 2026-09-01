#ifndef HPP_GUARD_PRAXIS_CONFIG_BINDING_H
#define HPP_GUARD_PRAXIS_CONFIG_BINDING_H

#include "praxis/config/error.h"
#include "praxis/config/store.h"
#include "praxis/config/writer.h"
#include "praxis/config/document.h"
#include "praxis/config/declaration.h"
#include "praxis/config/configurable.h"

#include "praxis/compat/expected.h"

#include <span>
#include <vector>

namespace praxis::config {

// One document, the keyspace it is read against and what it is expected to carry, handed in by
// whoever composes the thing that owns it. The library resolves the location it is given and knows
// none of its own.
struct binding
{
    declaration shape;
    location at;
    expectation carries;
};

outcome load_or_defaults(const binding &bound);

// The edits every shown implementor stands for, in the order they are shown. A null entry
// contributes none.
std::vector<edit> shown_edits(std::span<const configurable *const> shown, const document &carried);

// Whether anything the shown implementors stand for still has to reach `carried` -- the question
// a composition is asked when it is left.
bool anything_unsaved(std::span<const configurable *const> shown, const document &carried);

expected<void, error> save(const binding &bound, std::span<const edit> changes);

}

#endif
