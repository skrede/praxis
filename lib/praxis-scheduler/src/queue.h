#ifndef HPP_GUARD_PRAXIS_SCHEDULER_QUEUE_H
#define HPP_GUARD_PRAXIS_SCHEDULER_QUEUE_H

#include "praxis/scheduler/task.h"
#include "praxis/scheduler/clock.h"

#include "praxis/compat/detail/callable.h"

#include <vector>
#include <cstddef>
#include <cstdint>
#include <variant>
#include <optional>

namespace praxis::scheduler {

using scheduled_task = std::variant<sampled_task, stepped_task, one_shot_task>;

struct armed_task
{
    std::uint64_t id;
    scheduled_task work;

    time_point due() const;
    task_counters counters() const;
    bool repeats() const
    {
        return !std::holds_alternative<one_shot_task>(work);
    }

    bool sampled() const
    {
        return std::holds_alternative<sampled_task>(work);
    }

    void arm(time_point now);
    void run(time_point now);
};

struct discarded_work
{
    std::size_t ready_head;
    std::vector<armed_task> armed;
    std::vector<detail::move_only_function<void()>> ready;

    std::size_t ready_count() const
    {
        return ready.size() - ready_head;
    }
};

// One strand's serialized work: what has been posted and is ready, and what is due later. Every
// operation here is a plain operation on this record and takes nothing; what serializes access to
// one is whoever holds it. The nearest deadline is kept at the front of the heap and repeated in
// earliest(), so an advance compares one time point rather than scanning.
class strand_queue
{
public:
    strand_queue();

    bool retired() const
    {
        return m_retired;
    }

    bool ready() const
    {
        return m_head < m_ready.size();
    }

    bool occupied() const
    {
        return m_occupied;
    }

    bool select_ready(time_point now);
    time_point earliest() const;
    time_point delivery_time(const armed_task &entry, time_point now);

    std::vector<armed_task> retire();
    discarded_work evacuate();
    void occupy();
    void release();

    void push(detail::move_only_function<void()> &&work);
    detail::move_only_function<void()> take();

    void arm(armed_task &&entry);
    armed_task take_due();
    bool contains(std::uint64_t id) const;
    std::optional<task_counters> counters(std::uint64_t id) const;
    std::optional<armed_task> drop(std::uint64_t id, time_point now);

private:
    bool m_retired;
    bool m_occupied;
    std::size_t m_head;
    time_point m_earliest;
    time_point m_sample_due;
    time_point m_sample_time;
    std::uint32_t m_ready_streak;
    std::vector<armed_task> m_armed;
    std::vector<detail::move_only_function<void()>> m_ready;

    void reseat();
};

}

#endif
