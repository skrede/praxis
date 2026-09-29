#ifndef HPP_GUARD_PRAXIS_MANIPULATOR_SUPPLIED_CHAIN_H
#define HPP_GUARD_PRAXIS_MANIPULATOR_SUPPLIED_CHAIN_H

#include "praxis/rigid_motion/types.h"

#include <string>
#include <cstdint>

namespace praxis::manipulator {

// What keeps a supplied chain from being folded: a screw count that is not the rendered arm's joint
// count, a slot the chain is built or folded through that holds its default, or a joint whose screw
// the fold refused.
enum class withheld_cause : std::uint8_t
{
    joint_count,
    unbound_slot,
    refused
};

// Why a supplied chain is not folded, stated as a sentence a reader can be shown.
struct withheld_chain
{
    withheld_cause cause;
    std::string reason;
};

// Where a supplied chain ends, in the model's root-link frame: the flange at
// FK(theta) = e^[S1]theta1 ... e^[Sn]thetan M, and the tool at that flange times the tool offset the
// arm publishes. Lynch & Park, Modern Robotics, section 4.1.1.
struct chain_end
{
    transform flange;
    transform tool;
};

}

#endif
