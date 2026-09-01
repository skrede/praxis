#ifndef HPP_GUARD_PRAXIS_SCHEDULER_MARKER_H
#define HPP_GUARD_PRAXIS_SCHEDULER_MARKER_H

#include "praxis/scheduler/strand.h"

#include <cstdint>

namespace praxis::scheduler {

class pool;

// Which strand a handler runs under, kept as an intrusive stack rather than as one value, so that a
// handler entered from inside another handler on the same thread is representable at all. A node
// lives in the frame that installs it, so nothing allocates and nothing has to be unwound by hand.
class running_marker
{
public:
    running_marker(const pool &owner, strand_id on, std::uint64_t task);

    running_marker(const running_marker &)            = delete;
    running_marker(running_marker &&)                 = delete;
    running_marker &operator=(const running_marker &) = delete;
    running_marker &operator=(running_marker &&)      = delete;

    ~running_marker();

    static bool any(const pool &owner);
    static bool active(const pool &owner, strand_id on);
    static bool running_task(const pool &owner, std::uint64_t task);

private:
    strand_id m_id;
    const pool &m_owner;
    std::uint64_t m_task;
    running_marker *m_previous;
};

}

#endif
