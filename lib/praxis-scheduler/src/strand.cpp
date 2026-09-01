#include "praxis/scheduler/strand.h"
#include "praxis/scheduler/scheduler.h"

#include "pool.h"

#include <memory>
#include <optional>
#include <utility>

namespace praxis::scheduler {

strand::strand()
        : m_id(strand_id{0})
        , m_owner(nullptr)
{
}

strand::strand(scheduler &owner, strand_id id)
        : m_id(id)
        , m_owner(&owner)
{
}

bool strand::valid() const
{
    return m_owner != nullptr;
}

bool strand::running_here() const
{
    return m_owner != nullptr && m_owner->running_here(m_id);
}

strand_id strand::id() const
{
    return m_id;
}

task_handle::task_handle()
        : m_id(0)
        , m_owner()
{
}

task_handle::task_handle(const std::shared_ptr<pool> &owner, std::uint64_t id)
        : m_id(id)
        , m_owner(owner)
{
}

task_handle::task_handle(task_handle &&other) noexcept
        : m_id(other.m_id)
        , m_owner(std::move(other.m_owner))
{
    other.m_id = 0;
    other.m_owner.reset();
}

task_handle &task_handle::operator=(task_handle &&other) noexcept
{
    if(this != &other)
    {
        cancel();
        m_id       = other.m_id;
        m_owner    = std::move(other.m_owner);
        other.m_id = 0;
        other.m_owner.reset();
    }

    return *this;
}

task_handle::~task_handle()
{
    cancel();
}

bool task_handle::valid() const
{
    return !m_owner.expired();
}

bool task_handle::active() const
{
    const std::shared_ptr<pool> owner = m_owner.lock();

    return owner != nullptr && owner->active(m_id);
}

task_counters task_handle::counters() const
{
    const std::shared_ptr<pool> owner = m_owner.lock();
    if(owner == nullptr)
        return task_counters{0, 0, seconds::zero(), std::nullopt};

    return owner->counters(m_id);
}

void task_handle::cancel()
{
    const std::shared_ptr<pool> owner = m_owner.lock();
    m_owner.reset();
    const std::uint64_t id = std::exchange(m_id, 0);
    if(owner != nullptr)
        owner->cancel(id);
}

expected<void, rejection> task_handle::join()
{
    const std::shared_ptr<pool> owner = m_owner.lock();
    if(owner == nullptr)
        return {};

    const expected<void, rejection> joined = owner->join(m_id);
    if(!joined.has_value())
        return joined;

    m_owner.reset();
    m_id = 0;

    return {};
}

expected<void, rejection> strand::post(detail::move_only_function<void()> work) const
{
    if(m_owner == nullptr)
        return unexpected(rejection::unknown_strand);

    return m_owner->post(m_id, std::move(work));
}

task_handle strand::after(step_period delay, detail::move_only_function<void()> run) const
{
    if(m_owner == nullptr || run == nullptr)
        return task_handle();

    return m_owner->after(m_id, delay, std::move(run));
}

task_handle strand::every(step_period period, overrun policy, detail::move_only_function<void(step_delta)> work, replay_bound bound) const
{
    if(m_owner == nullptr || work == nullptr || bound.ticks == 0)
        return task_handle();

    return m_owner->every(m_id, period, policy, std::move(work), bound);
}

task_handle strand::sample(step_period period, detail::move_only_function<void(sample_time)> work) const
{
    if(m_owner == nullptr || work == nullptr || !(period.value > seconds::zero()))
        return task_handle();

    return m_owner->sample(m_id, period, std::move(work));
}

}
