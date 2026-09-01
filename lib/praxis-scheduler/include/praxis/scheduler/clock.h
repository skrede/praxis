#ifndef HPP_GUARD_PRAXIS_SCHEDULER_CLOCK_H
#define HPP_GUARD_PRAXIS_SCHEDULER_CLOCK_H

#include <chrono>

namespace praxis::scheduler {

using seconds    = std::chrono::duration<double>;
using time_point = std::chrono::steady_clock::time_point;

time_point steady_now();

// The clock is a slot rather than a template parameter: a parameter would put the clock's type in
// the signature of everything that holds a scheduler, and what an overrun policy does with an
// interval cannot be observed at all unless the interval can be dictated.
struct clock_source
{
    time_point (*now)() = &steady_now;
};

}

#endif
