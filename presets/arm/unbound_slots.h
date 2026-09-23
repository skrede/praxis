#ifndef HPP_GUARD_PRAXIS_PRESETS_ARM_UNBOUND_SLOTS_H
#define HPP_GUARD_PRAXIS_PRESETS_ARM_UNBOUND_SLOTS_H

#include "praxis/extension/coverage.h"
#include "praxis/extension/slot_set.h"

#include <array>
#include <string>
#include <cstddef>

namespace praxis::presets {

// The names come from the same descriptor table the coverage report reads, so a slot renamed in one
// place is named the same way here.
template<typename Ops, slot_enumeration Slot, std::size_t N>
std::string unbound_among(const Ops &names, const basic_slot_set<Slot> &inert, const std::array<Slot, N> &needed)
{
    const capability_view described = view_of(names);

    std::string listed;
    for(const Slot wanted : needed)
        if(inert.contains(wanted))
        {
            if(!listed.empty())
                listed += ", ";
            listed += slot_name(described, static_cast<std::size_t>(wanted));
        }

    return listed;
}

}

#endif
