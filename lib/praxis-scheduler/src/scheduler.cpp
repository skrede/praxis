#include "praxis/scheduler/scheduler.h"

#include "pool.h"
#include "marker.h"

#include <memory>
#include <thread>
#include <utility>

namespace praxis::scheduler {

scheduler::scheduler(std::uint32_t workers)
        : scheduler(workers, clock_source{})
{
}

std::uint32_t default_workers()
{
    const std::uint32_t available = std::thread::hardware_concurrency();

    return available < 3 ? 1 : 2;
}

scheduler::scheduler(std::uint32_t workers, clock_source clock)
        : m_self(this)
        , m_simulation()
        , m_pool(std::make_shared<pool>(workers, clock))
{
    const expected<strand, rejection> made = make_strand();
    if(made.has_value())
        m_simulation = *made;
}

scheduler::~scheduler() = default;

strand scheduler::main_strand() const
{
    return strand(*m_self, m_pool->main());
}

strand scheduler::simulation_strand() const
{
    return m_simulation;
}

expected<strand, rejection> scheduler::make_strand()
{
    const expected<strand_id, rejection> made = m_pool->make();
    if(!made.has_value())
        return unexpected(made.error());

    return strand(*m_self, *made);
}

expected<void, rejection> scheduler::retire_strand(strand target, detail::move_only_function<void()> acknowledgment)
{
    if(!target.valid() || target.m_owner != this)
        return unexpected(rejection::unknown_strand);

    return m_pool->retire(target.m_id, std::move(acknowledgment));
}

bool scheduler::step()
{
    return m_pool->step();
}

expected<void, rejection> scheduler::drain()
{
    return m_pool->drain();
}

void scheduler::run()
{
    m_pool->run();
}

void scheduler::stop()
{
    m_pool->stop();
}

bool scheduler::running_here(strand_id on) const
{
    return m_pool->known(on) && running_marker::active(*m_pool, on);
}

expected<void, rejection> scheduler::post(strand_id on, detail::move_only_function<void()> work)
{
    return m_pool->post(on, std::move(work));
}

task_handle scheduler::after(strand_id on, step_period delay, detail::move_only_function<void()> run)
{
    const std::uint64_t id = m_pool->arm(on, one_shot_task(delay, std::move(run)));
    if(id == 0)
        return task_handle();

    return task_handle(m_pool, id);
}

task_handle scheduler::every(strand_id on, step_period period, overrun policy, detail::move_only_function<void(step_delta)> work, replay_bound bound)
{
    const std::uint64_t id = m_pool->arm(on, stepped_task(period, policy, std::move(work), bound));
    if(id == 0)
        return task_handle();

    return task_handle(m_pool, id);
}

task_handle scheduler::sample(strand_id on, step_period period, detail::move_only_function<void(sample_time)> work)
{
    if(!(period.value > seconds::zero()))
        return task_handle();

    const std::uint64_t id = m_pool->arm(on, sampled_task(period, std::move(work)));
    if(id == 0)
        return task_handle();

    return task_handle(m_pool, id);
}

}
