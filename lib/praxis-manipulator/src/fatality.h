#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_FATALITY_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_FATALITY_H

#include "praxis/extension/refusal.h"

#include <functional>
#include <string_view>

namespace praxis::manipulator {

// The one place that decides what becomes of a composition a slot has refused, so a command written
// later inherits the rule instead of carrying a copy of it. What is fatal follows the kind of the
// refusal and never the call site or the slot: a request the binding does not serve and an input
// ill-formed for the mathematics both leave the composition unable to answer for itself, and it asks
// to be unloaded. An unbound binding and an answer that does not exist change nothing here. The
// caller reports what it received; this reports only the unloading, and asking is all it does --
// what becomes of the composition is the composition's own decision. A fatal kind of
// composition-wide standing asks nothing: a refusal true at every configuration of the composition,
// or across a whole region of them, would unload it the moment it was named, and the composition
// still answers everything else it was composed for.
void tear_down_if_fatal(std::string_view named, refusal reason, refusal_standing standing, const std::function<void()> &ask_unload);

}

#endif
