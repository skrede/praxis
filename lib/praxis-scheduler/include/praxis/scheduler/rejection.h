#ifndef HPP_GUARD_PRAXIS_SCHEDULER_REJECTION_H
#define HPP_GUARD_PRAXIS_SCHEDULER_REJECTION_H

#include <cstdint>

namespace praxis::scheduler {

// This module answers over two channels and the division is deliberate: a state the caller is
// expected to handle is returned through this enumeration, and a programming error throws.
// strand_retired: the strand admits no further work and never will again. scheduler_stopped: the
// scheduler has been stopped and runs nothing more. empty_work: the callable offered holds no
// target. reentrant_drain: a drain was asked for from inside a handler the scheduler is running.
// unknown_strand: the handle names no strand of this scheduler. strand_limit_reached: no distinct
// strand identity remains representable. reentrant_join: a blocking join was asked for from inside a
// handler the scheduler is running.
enum class rejection : std::uint8_t
{
    strand_retired,
    scheduler_stopped,
    empty_work,
    reentrant_drain,
    unknown_strand,
    strand_limit_reached,
    reentrant_join
};

}

#endif
