#ifndef HPP_GUARD_PRAXIS_SCHEDULER_OVERRUN_H
#define HPP_GUARD_PRAXIS_SCHEDULER_OVERRUN_H

#include "praxis/scheduler/clock.h"

#include <cstdint>
#include <optional>

namespace praxis::scheduler {

// Both are honest about the interval that actually passed and differ only in whether it is
// decomposed. catch_up: the handler is called once per period the interval covers, each call handed
// that period. drop: the handler is called once, handed the whole of the interval.
enum class overrun : std::uint8_t
{
    catch_up,
    drop
};

// A registration's ceiling on the periods one service replays under catch_up. Zero admits nothing
// and is refused. The deadline still advances over the whole interval that passed, so what the
// bound leaves undelivered is forgiven rather than owed again at the next service, and is counted
// as dropped.
struct replay_bound
{
    std::uint64_t ticks;
};

// The bound a registration carries when it names none of its own.
inline constexpr std::uint64_t catch_up_limit = 8192;

// What a task has done with the deadlines that came due for it. ran plus dropped is every whole
// period that elapsed, whichever policy applied and whether or not the bound forgave part of it,
// and the largest lateness is measured from the deadline a fire was scheduled for. The policy is
// absent on the admissions that carry none.
struct task_counters
{
    std::uint64_t ran;
    std::uint64_t dropped;
    seconds worst_lateness;
    std::optional<overrun> policy;
};

}

#endif
