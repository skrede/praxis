#include "deadline.h"

#include <cmath>
#include <chrono>
#include <cstdint>
#include <type_traits>

namespace praxis::scheduler {

namespace {

using unsigned_rep = std::make_unsigned_t<time_point::duration::rep>;

unsigned_rep since_epoch(time_point value)
{
    return static_cast<unsigned_rep>(value.time_since_epoch().count());
}

time_point::duration span(time_point::duration quantum, std::uint64_t periods)
{
    using duration   = time_point::duration;
    const auto step  = static_cast<std::uint64_t>(quantum.count());
    const auto limit = static_cast<std::uint64_t>(duration::max().count());
    if(step == 0 || periods == 0)
        return duration::zero();

    return periods > limit / step ? duration::max() : duration{static_cast<duration::rep>(periods * step)};
}

}

time_point::duration clock_duration(seconds value, bool allow_zero)
{
    using duration          = time_point::duration;
    using floating_duration = std::chrono::duration<long double, duration::period>;

    if(!std::isfinite(value.count()) || value < seconds::zero())
        return duration::min();
    if(value == seconds::zero())
        return allow_zero ? duration::zero() : duration::min();
    if(value < seconds{duration{1}})
        return duration::min();

    const long double ticks = std::chrono::duration_cast<floating_duration>(value).count();
    const long double limit = static_cast<long double>(duration::max().count()) + 1.0L;
    if(!std::isfinite(ticks) || ticks >= limit)
        return duration::min();

    return duration{ticks < 1.0L ? 1 : static_cast<duration::rep>(ticks)};
}

time_point advanced(time_point from, time_point::duration by)
{
    using duration             = time_point::duration;
    const duration::rep origin = from.time_since_epoch().count();
    if(by < duration::zero())
        return time_point::max();
    if(origin > 0 && by.count() > duration::max().count() - origin)
        return time_point::max();

    return from + by;
}

time_point advanced(time_point from, seconds by, bool allow_zero)
{
    return advanced(from, clock_duration(by, allow_zero));
}

time_point advanced(time_point from, time_point::duration quantum, std::uint64_t periods)
{
    return quantum < time_point::duration::zero() ? time_point::max() : advanced(from, span(quantum, periods));
}

time_point next_sample_deadline(time_point due, time_point now, seconds period)
{
    using duration = time_point::duration;

    const duration quantum = clock_duration(period, false);
    if(quantum < duration{1})
        return time_point::max();
    if(now < due)
        return due;

    const unsigned_rep late = since_epoch(now) - since_epoch(due);
    const unsigned_rep step = static_cast<unsigned_rep>(quantum.count());
    const unsigned_rep next = late % step == 0 ? step : step - late % step;
    const unsigned_rep room = static_cast<unsigned_rep>(duration::max().count()) - since_epoch(now);

    return next > room ? time_point::max() : advanced(now, duration{static_cast<duration::rep>(next)});
}

std::uint64_t owed_periods(time_point due, time_point now, time_point::duration quantum)
{
    if(now < due || quantum < time_point::duration{1})
        return 1;

    const unsigned_rep late = since_epoch(now) - since_epoch(due);

    return 1 + static_cast<std::uint64_t>(late / static_cast<unsigned_rep>(quantum.count()));
}

seconds lateness(time_point due, time_point now)
{
    using duration           = time_point::duration;
    const unsigned_rep limit = static_cast<unsigned_rep>(duration::max().count());
    if(now <= due)
        return seconds::zero();

    const unsigned_rep late = since_epoch(now) - since_epoch(due);

    return seconds{duration{static_cast<duration::rep>(late < limit ? late : limit)}};
}

}
