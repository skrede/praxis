#include "pool.h"

#include "marker.h"

#include <mutex>
#include <optional>
#include <algorithm>

namespace praxis::scheduler {

void pool::cancel(std::uint64_t id)
{
    withdraw(id, withdrawal::marked);
}

expected<void, rejection> pool::join(std::uint64_t id)
{
    if(running_marker::any(*this))
        return unexpected(rejection::reentrant_join);

    withdraw(id, withdrawal::joined);

    return {};
}

// The dropped work is carried out of the locked region before it is destroyed, because a callable's
// destructor reaches whatever the handler captured and that must not run under the pool's mutex.
void pool::withdraw(std::uint64_t id, withdrawal mode)
{
    std::optional<armed_task> discarded;
    const time_point now = m_clock.now();
    bool notify          = false;
    {
        std::unique_lock held(m_mutex);
        for(strand_queue &queue : m_strands)
        {
            discarded = queue.drop(id, now);
            if(discarded.has_value())
                break;
        }
        if(!discarded.has_value() && mark_cancelled(id) && mode == withdrawal::joined)
            await_completion(held, id);
        notify = publish();
    }

    if(notify)
        m_wakeup.notify_all();
}

// True when an invocation was in flight and has now been marked, so the strand refuses its rearm.
bool pool::mark_cancelled(std::uint64_t id)
{
    const auto found = std::find_if(m_running.begin(), m_running.end(), [id](const running_entry &entry) { return entry.id == id; });
    if(found == m_running.end())
        return false;

    found->cancelled = true;

    return true;
}

// The caller must already have refused a join made from inside a handler of this pool: a thread that
// parks here while carrying an invocation of its own is waiting on work only it can finish.
void pool::await_completion(std::unique_lock<std::mutex> &held, std::uint64_t id)
{
    m_completion.wait(held, [this, id] { return !in_flight(id); });
}

bool pool::in_flight(std::uint64_t id) const
{
    return std::any_of(m_running.begin(), m_running.end(), [id](const running_entry &entry) { return entry.id == id; });
}

bool pool::active(std::uint64_t id) const
{
    const std::lock_guard held(m_mutex);
    if(in_flight(id))
        return true;

    return std::any_of(m_strands.begin(), m_strands.end(), [id](const strand_queue &queue) { return queue.contains(id); });
}

task_counters pool::counters(std::uint64_t id) const
{
    const std::lock_guard held(m_mutex);
    const auto running = std::find_if(m_running.begin(), m_running.end(), [id](const running_entry &entry) { return entry.id == id; });
    if(running != m_running.end())
        return running->tally;

    for(const strand_queue &queue : m_strands)
        if(const std::optional<task_counters> found = queue.counters(id); found.has_value())
            return *found;

    return task_counters{0, 0, seconds::zero(), std::nullopt};
}

}
