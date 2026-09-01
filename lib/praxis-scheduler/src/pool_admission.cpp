#include "pool.h"

#include <mutex>
#include <limits>
#include <vector>
#include <utility>

namespace praxis::scheduler {

std::size_t seat(strand_id on)
{
    return static_cast<std::size_t>(on);
}

strand_id pool::main() const
{
    return strand_id{0};
}

bool pool::known(strand_id on) const
{
    const std::lock_guard held(m_mutex);

    return seat(on) < m_strands.size();
}

expected<strand_id, rejection> pool::make()
{
    const std::lock_guard held(m_mutex);
    if(m_stopped)
        return unexpected(rejection::scheduler_stopped);
    if(m_strands.size() > std::numeric_limits<std::uint32_t>::max())
        return unexpected(rejection::strand_limit_reached);

    m_strands.emplace_back();

    return strand_id{static_cast<std::uint32_t>(m_strands.size() - 1)};
}

expected<void, rejection> pool::retire(strand_id on, detail::move_only_function<void()> acknowledgment)
{
    std::vector<armed_task> discarded;
    bool notify = false;
    {
        const std::lock_guard held(m_mutex);
        if(seat(on) >= m_strands.size())
            return unexpected(rejection::unknown_strand);
        if(m_stopped)
            return unexpected(rejection::scheduler_stopped);
        if(m_strands[seat(on)].retired())
            return unexpected(rejection::strand_retired);

        discarded = m_strands[seat(on)].retire();
        if(acknowledgment != nullptr)
            admit(on, std::move(acknowledgment));
        notify = publish();
    }

    if(notify)
        m_wakeup.notify_all();

    return {};
}

expected<void, rejection> pool::post(strand_id on, detail::move_only_function<void()> work)
{
    if(work == nullptr)
        return unexpected(rejection::empty_work);

    bool notify = false;
    {
        const std::lock_guard held(m_mutex);
        if(seat(on) >= m_strands.size())
            return unexpected(rejection::unknown_strand);
        if(m_stopped)
            return unexpected(rejection::scheduler_stopped);
        if(m_strands[seat(on)].retired())
            return unexpected(rejection::strand_retired);

        admit(on, std::move(work));
        notify = publish();
    }

    if(notify)
        m_wakeup.notify_all();

    return {};
}

std::uint64_t pool::arm(strand_id on, scheduled_task work)
{
    armed_task entry{0, std::move(work)};
    std::uint64_t id = 0;
    bool notify      = false;

    {
        const std::lock_guard held(m_mutex);
        if(m_stopped || m_sequence == std::numeric_limits<std::uint64_t>::max() || seat(on) >= m_strands.size() || m_strands[seat(on)].retired())
            return 0;

        entry.arm(m_clock.now());
        if(entry.due() == time_point::max())
            return 0;
        id       = ++m_sequence;
        entry.id = id;
        m_strands[seat(on)].arm(std::move(entry));
        notify = publish();
    }

    if(notify)
        m_wakeup.notify_all();

    return id;
}

void pool::admit(strand_id on, detail::move_only_function<void()> &&work)
{
    m_strands[seat(on)].push(std::move(work));
    ++m_outstanding;
}

bool pool::publish()
{
    ++m_generation;
    if(m_waiting == 0 || m_signalled)
        return false;

    m_signalled = true;

    return true;
}

}
