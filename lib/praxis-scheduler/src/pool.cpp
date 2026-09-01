#include "pool.h"

#include "marker.h"

#include <mutex>
#include <thread>

namespace praxis::scheduler {

pool::pool(std::uint32_t workers, clock_source clock)
        : m_stopped(false)
        , m_threaded(workers != 0)
        , m_signalled(false)
        , m_clock(clock)
        , m_waiting(0)
        , m_sequence(0)
        , m_mutex()
        , m_generation(0)
        , m_inline_cursor(0)
        , m_worker_cursor(1)
        , m_outstanding(0)
        , m_wakeup()
        , m_completion()
        , m_workers()
        , m_strands(1)
        , m_running()
{
    m_workers.reserve(workers);
    worker_start_guard started(*this);
    for(std::uint32_t index = 0; index < workers; ++index)
        m_workers.emplace_back([this] { attend(true); });
    started.complete();
}

// Setting the stopped state under the mutex closes the gap between a worker's predicate check and
// its block; the notification can then occur after the lock is released.
pool::~pool()
{
    stop();
    join_workers();
}

pool::worker_start_guard::worker_start_guard(pool &owner)
        : m_active(true)
        , m_owner(owner)
{
}

pool::worker_start_guard::~worker_start_guard()
{
    if(m_active)
    {
        m_owner.stop();
        m_owner.join_workers();
    }
}

void pool::worker_start_guard::complete()
{
    m_active = false;
}

void pool::join_workers()
{
    for(std::thread &worker : m_workers)
        if(worker.joinable())
            worker.join();
}

bool pool::step()
{
    return advance(false);
}

expected<void, rejection> pool::drain()
{
    if(running_marker::any(*this))
        return unexpected(rejection::reentrant_drain);
    while(true)
    {
        if(step())
            continue;

        std::unique_lock held(m_mutex);
        if(m_stopped)
            return unexpected(rejection::scheduler_stopped);
        if(quiet())
            return {};
        if(pending(span_held(false)))
            continue;

        const std::uint64_t observed = m_generation;
        ++m_waiting;
        m_signalled = false;
        while(!m_stopped && m_generation == observed)
            m_wakeup.wait(held);
        --m_waiting;
    }
}

void pool::run()
{
    attend(false);
}

void pool::stop()
{
    std::size_t count = 0;
    {
        const std::lock_guard held(m_mutex);
        m_stopped   = true;
        m_signalled = true;
        ++m_generation;
        count = m_strands.size();
    }

    m_wakeup.notify_all();
    for(std::size_t index = 0; index < count; ++index)
    {
        discarded_work discarded = evacuate(index);
    }
}

discarded_work pool::evacuate(std::size_t index)
{
    discarded_work discarded{0, {}, {}};
    {
        const std::lock_guard held(m_mutex);
        discarded = m_strands[index].evacuate();
        m_outstanding -= discarded.ready_count();
        ++m_generation;
    }

    return discarded;
}

bool pool::stopped() const
{
    const std::lock_guard held(m_mutex);

    return m_stopped;
}

void pool::attend(bool worker)
{
    while(!stopped())
    {
        if(advance(worker))
            continue;

        std::unique_lock held(m_mutex);
        park(held, worker);
    }
}

bool pool::advance(bool worker)
{
    std::unique_lock held(m_mutex);
    const served range = span_held(worker);
    if(m_stopped || range.first == range.limit)
        return false;

    std::size_t &cursor = worker ? m_worker_cursor : m_inline_cursor;
    if(cursor < range.first || cursor >= range.limit)
        cursor = range.first;
    const std::size_t width = range.limit - range.first;
    for(std::size_t offset = 0; offset < width; ++offset)
    {
        const std::size_t index = range.first + (cursor - range.first + offset) % width;
        const time_point now    = m_clock.now();
        const std::size_t next  = index + 1 == range.limit ? range.first : index + 1;
        if(advance_one(held, strand_id{static_cast<std::uint32_t>(index)}, now, cursor, next))
            return true;
    }

    return false;
}

}
