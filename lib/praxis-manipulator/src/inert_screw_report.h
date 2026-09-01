#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_INERT_SCREW_REPORT_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_INERT_SCREW_REPORT_H

#include "praxis/rigid_motion/screw.h"
#include "praxis/rigid_motion/slots.h"

#include <string_view>

namespace praxis::manipulator {

// How the screw capability's own descriptor table spells one of its slots. The value is read for
// the table describing it and not for what it binds, so every composition of it answers the same
// name.
std::string_view screw_slot_name(const rigid_motion::screw_ops &described, rigid_motion::screw_slot which);

// Whether the composition left `which` at its default, said once for as long as that stands and
// followed by the clause the caller supplies. The latch is the caller's and is cleared when the
// condition lifts, so a caller running every frame says it once rather than once a frame.
bool inert_and_reported(const rigid_motion::screw_ops &described, const rigid_motion::screw_slot_set &inert, rigid_motion::screw_slot which, std::string_view so_that, bool &reported);

}

#endif
