#include "praxis/scheduler/task.h"

#include "deadline.h"

#include <chrono>
#include <cstdint>
#include <optional>
#include <utility>
#include <algorithm>

namespace praxis::scheduler {

time_point steady_now()
{
    return std::chrono::steady_clock::now();
}

sampled_task::sampled_task(step_period period, detail::move_only_function<void(sample_time)> sample)
        : m_due()
        , m_period(period)
        , m_counters{0, 0, seconds::zero(), std::nullopt}
        , m_sample_cb(std::move(sample))
{
}

time_point sampled_task::due() const
{
    return m_due;
}

task_counters sampled_task::counters() const
{
    return m_counters;
}

void sampled_task::arm(time_point now)
{
    m_due = next_sample_deadline(now, now, m_period.value);
}

void sampled_task::run(time_point now)
{
    m_counters.worst_lateness = std::max(m_counters.worst_lateness, lateness(m_due, now));
    ++m_counters.ran;
    m_due = next_sample_deadline(m_due, now, m_period.value);
    m_sample_cb(sample_time{seconds{now.time_since_epoch()}});
}

stepped_task::stepped_task(step_period period, overrun policy, detail::move_only_function<void(step_delta)> work, replay_bound bound)
        : m_due()
        , m_last()
        , m_bound(bound)
        , m_period(period)
        , m_counters{0, 0, seconds::zero(), policy}
        , m_step_cb(std::move(work))
{
}

time_point stepped_task::due() const
{
    return m_due;
}

task_counters stepped_task::counters() const
{
    return m_counters;
}

void stepped_task::arm(time_point now)
{
    m_last = now;
    m_due  = advanced(now, m_period.value, true);
}

void stepped_task::run(time_point now)
{
    const time_point::duration quantum = clock_duration(m_period.value, true);
    const std::uint64_t owed           = owed_periods(m_due, now, quantum);

    m_counters.worst_lateness = std::max(m_counters.worst_lateness, lateness(m_due, now));
    if(m_counters.policy == overrun::drop || quantum < time_point::duration{1})
        deliver_whole(now, owed, quantum);
    else
        replay(owed, quantum);
}

// The bound forgives what it refuses rather than deferring it: both deadline and elapsed-time
// origin advance over every period that passed, and the difference is counted as dropped.
void stepped_task::replay(std::uint64_t owed, time_point::duration quantum)
{
    const std::uint64_t delivered = std::min(owed, m_bound.ticks);
    m_last                        = advanced(m_last, quantum, owed);
    m_due                         = advanced(m_due, quantum, owed);
    m_counters.ran += delivered;
    m_counters.dropped += owed - delivered;

    for(std::uint64_t tick = 0; tick < delivered; ++tick)
        m_step_cb(step_delta{m_period.value});
}

void stepped_task::deliver_whole(time_point now, std::uint64_t owed, time_point::duration quantum)
{
    const seconds elapsed = now - m_last;
    m_due                 = advanced(m_due, quantum, owed);
    m_last                = now;
    ++m_counters.ran;
    m_counters.dropped += owed - 1;
    m_step_cb(step_delta{elapsed});
}

one_shot_task::one_shot_task(step_period delay, detail::move_only_function<void()> run)
        : m_due()
        , m_delay(delay)
        , m_counters{0, 0, seconds::zero(), std::nullopt}
        , m_run_cb(std::move(run))
{
}

time_point one_shot_task::due() const
{
    return m_due;
}

task_counters one_shot_task::counters() const
{
    return m_counters;
}

void one_shot_task::arm(time_point now)
{
    m_due = advanced(now, m_delay.value, true);
}

void one_shot_task::run(time_point now)
{
    m_counters.worst_lateness = lateness(m_due, now);
    ++m_counters.ran;
    m_run_cb();
}

}
