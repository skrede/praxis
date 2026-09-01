#include "pool.h"

#include <mutex>
#include <algorithm>
#include <type_traits>

namespace praxis::scheduler {

namespace {

// A deadline is a reading of the injected clock, and a timed wait is against the real steady clock.
// The two are unrelated domains whenever a clock is injected, so what is carried over is the
// remaining interval rather than the deadline itself.
time_point real_deadline(time_point until, time_point now)
{
    using duration     = time_point::duration;
    using unsigned_rep = std::make_unsigned_t<duration::rep>;

    const time_point real = steady_now();
    if(until <= now)
        return real;

    const unsigned_rep span = static_cast<unsigned_rep>(until.time_since_epoch().count()) - static_cast<unsigned_rep>(now.time_since_epoch().count());
    const unsigned_rep room = static_cast<unsigned_rep>(duration::max().count()) - static_cast<unsigned_rep>(real.time_since_epoch().count());

    return span > room ? time_point::max() : real + duration{static_cast<duration::rep>(span)};
}

}

void pool::park(std::unique_lock<std::mutex> &held, bool worker)
{
    const served range = span_held(worker);
    if(m_stopped || pending(range))
        return;

    const time_point now = m_clock.now();
    time_point until     = time_point::max();
    for(std::size_t index = range.first; index < range.limit; ++index)
        if(!m_strands[index].occupied())
            until = std::min(until, m_strands[index].earliest());

    ++m_waiting;
    m_signalled = false;
    if(until == time_point::max())
        m_wakeup.wait(held);
    else
        m_wakeup.wait_until(held, real_deadline(until, now));
    --m_waiting;
}

served pool::span_held(bool worker) const
{
    const std::size_t count = m_strands.size();
    if(worker)
        return served{1, count};

    return served{0, m_threaded ? 1 : count};
}

bool pool::pending(served range) const
{
    const time_point now = m_clock.now();
    for(std::size_t index = range.first; index < range.limit; ++index)
        if(!m_strands[index].occupied() && (m_strands[index].ready() || m_strands[index].earliest() <= now))
            return true;

    return false;
}

bool pool::quiet() const
{
    return m_outstanding == 0 && !pending(served{0, m_strands.size()});
}

}
