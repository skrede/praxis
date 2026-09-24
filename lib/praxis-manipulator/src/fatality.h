#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_FATALITY_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_FATALITY_H

#include "praxis/extension/refusal.h"

#include <functional>
#include <string_view>

namespace praxis::manipulator {

// The one place that decides what becomes of a composition a slot has refused, so a command written
// later inherits the rule instead of carrying a copy of it. What is fatal follows the refusal's own
// three axes and never the call site or the slot: a request the binding does not serve and an input
// ill-formed for the mathematics both leave the composition unable to answer for itself, and it asks
// to be unloaded. An unbound binding and an answer that does not exist change nothing here. The
// caller reports what it received; this reports only the unloading, and asking is all it does --
// what becomes of the composition is the composition's own decision. A fatal kind of
// composition-wide standing asks nothing: a refusal true at every configuration of the composition,
// or across a whole region of them, would unload it the moment it was named, and the composition
// still answers everything else it was composed for. Neither does a request formed from a value an
// operator is editing: such a request is re-issued at every move of that value, so the ill-formed
// ones among them are what was asked for and say nothing about what answered.
void tear_down_if_fatal(std::string_view named, refusal reason, refusal_standing standing, request_origin formed_from, const std::function<void(std::string)> &ask_unload);

}

#endif
