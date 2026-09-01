#include "pool.h"

#include "marker.h"

#include <mutex>
#include <utility>
#include <algorithm>

namespace praxis::scheduler {

pool::service_guard::service_guard(pool &owner, std::unique_lock<std::mutex> &held, strand_id on, std::uint64_t running)
        : m_active(true)
        , m_owner(owner)
        , m_on(on)
        , m_running(running)
        , m_held(held)
{
}

pool::service_guard::~service_guard()
{
    if(m_active)
        m_owner.abandon(m_held, m_on, m_running);
}

void pool::service_guard::complete()
{
    m_active = false;
}

bool pool::advance_one(std::unique_lock<std::mutex> &held, strand_id on, time_point now, std::size_t &cursor, std::size_t next)
{
    if(m_stopped || seat(on) >= m_strands.size() || m_strands[seat(on)].occupied())
        return false;
    if(m_strands[seat(on)].select_ready(now))
    {
        cursor = next;
        return run_ready(held, on);
    }
    if(m_strands[seat(on)].earliest() <= now)
    {
        cursor = next;
        return run_due(held, on, now);
    }

    return false;
}

bool pool::run_ready(std::unique_lock<std::mutex> &held, strand_id on)
{
    if(!m_strands[seat(on)].ready())
        return false;

    detail::move_only_function<void()> work = m_strands[seat(on)].take();
    m_strands[seat(on)].occupy();
    service_guard completion(*this, held, on, 0);
    held.unlock();

    {
        const running_marker marker(*this, on, 0);
        work();
    }

    finish_ready(held, on);
    completion.complete();

    return true;
}

bool pool::run_due(std::unique_lock<std::mutex> &held, strand_id on, time_point now)
{
    m_running.reserve(m_running.size() + 1);
    armed_task entry = m_strands[seat(on)].take_due();
    m_running.push_back(running_entry{false, entry.id, entry.counters()});
    ++m_outstanding;
    m_strands[seat(on)].occupy();
    service_guard completion(*this, held, on, entry.id);
    const time_point delivered = m_strands[seat(on)].delivery_time(entry, now);
    held.unlock();

    {
        const running_marker marker(*this, on, entry.id);
        entry.run(delivered);
    }

    finish_due(held, on, entry);
    completion.complete();

    return true;
}

void pool::abandon(std::unique_lock<std::mutex> &held, strand_id on, std::uint64_t running)
{
    if(!held.owns_lock())
        held.lock();
    if(running != 0)
        std::erase_if(m_running, [running](const running_entry &entry) { return entry.id == running; });
    --m_outstanding;
    m_strands[seat(on)].release();
    const bool notify = publish();
    held.unlock();

    m_completion.notify_all();
    if(notify)
        m_wakeup.notify_all();
}

void pool::finish_ready(std::unique_lock<std::mutex> &held, strand_id on)
{
    held.lock();
    --m_outstanding;
    m_strands[seat(on)].release();
    const bool notify = publish();
    held.unlock();

    if(notify)
        m_wakeup.notify_all();
}

void pool::finish_due(std::unique_lock<std::mutex> &held, strand_id on, armed_task &entry)
{
    held.lock();
    const std::uint64_t id = entry.id;
    const auto found       = std::find_if(m_running.begin(), m_running.end(), [id](const running_entry &candidate) { return candidate.id == id; });
    const bool rearms      = found != m_running.end() && !found->cancelled;
    if(rearms && !m_stopped && !m_strands[seat(on)].retired() && entry.repeats() && entry.due() != time_point::max())
        m_strands[seat(on)].arm(std::move(entry));
    if(found != m_running.end())
        m_running.erase(found);
    --m_outstanding;
    m_strands[seat(on)].release();
    const bool notify = publish();
    held.unlock();

    m_completion.notify_all();
    if(notify)
        m_wakeup.notify_all();
}

}
