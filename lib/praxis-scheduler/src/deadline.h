#ifndef HPP_GUARD_PRAXIS_SCHEDULER_DEADLINE_H
#define HPP_GUARD_PRAXIS_SCHEDULER_DEADLINE_H

#include "praxis/scheduler/clock.h"

#include <cstdint>

namespace praxis::scheduler {

// A period is a floating-point count of seconds and a deadline is an integer count of clock ticks.
// Everything here converts between the two, counts in ticks so a period that is not a binary
// fraction of a second stays exact over long intervals, and saturates at the clock's terminal value
// rather than wrapping. duration::min() is the conversion's refusal.
time_point::duration clock_duration(seconds value, bool allow_zero);

time_point advanced(time_point from, time_point::duration by);

time_point advanced(time_point from, seconds by, bool allow_zero);

time_point advanced(time_point from, time_point::duration quantum, std::uint64_t periods);

time_point next_sample_deadline(time_point due, time_point now, seconds period);

// One, plus the periods that fit between the deadline and now: the deadlines that have come due.
std::uint64_t owed_periods(time_point due, time_point now, time_point::duration quantum);

seconds lateness(time_point due, time_point now);

}

#endif
