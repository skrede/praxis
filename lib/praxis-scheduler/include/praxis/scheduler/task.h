#ifndef HPP_GUARD_PRAXIS_SCHEDULER_TASK_H
#define HPP_GUARD_PRAXIS_SCHEDULER_TASK_H

#include "praxis/scheduler/clock.h"
#include "praxis/scheduler/overrun.h"
#include "praxis/scheduler/rejection.h"

#include "praxis/compat/detail/callable.h"
#include "praxis/compat/expected.h"

#include <memory>
#include <cstdint>

namespace praxis::scheduler {

class pool;
class scheduler;

struct sample_time
{
    seconds value;
};

struct step_delta
{
    seconds value;
};

struct step_period
{
    seconds value;
};

// The fastest step whoever services the strand can offer.
inline constexpr step_period every_step{seconds{0.0}};

class sampled_task
{
public:
    sampled_task(step_period period, detail::move_only_function<void(sample_time)> sample);

    time_point due() const;

    task_counters counters() const;

    void arm(time_point now);

    void run(time_point now);

private:
    time_point m_due;
    step_period m_period;
    task_counters m_counters;
    detail::move_only_function<void(sample_time)> m_sample_cb;
};

// Registration is the first elapsed-time origin. The overrun policy determines whether that
// interval is delivered whole or decomposed into period-sized callbacks, and the bound is the most
// periods one service decomposes it into.
class stepped_task
{
public:
    stepped_task(step_period period, overrun policy, detail::move_only_function<void(step_delta)> work, replay_bound bound = replay_bound{catch_up_limit});

    time_point due() const;

    task_counters counters() const;

    void arm(time_point now);

    void run(time_point now);

private:
    time_point m_due;
    time_point m_last;
    replay_bound m_bound;
    step_period m_period;
    task_counters m_counters;
    detail::move_only_function<void(step_delta)> m_step_cb;

    void replay(std::uint64_t owed, time_point::duration quantum);

    void deliver_whole(time_point now, std::uint64_t owed, time_point::duration quantum);
};

class one_shot_task
{
public:
    one_shot_task(step_period delay, detail::move_only_function<void()> run);

    time_point due() const;

    task_counters counters() const;

    void arm(time_point now);

    void run(time_point now);

private:
    time_point m_due;
    step_period m_delay;
    task_counters m_counters;
    detail::move_only_function<void()> m_run_cb;
};

// A registration receipt. Destruction and cancel() mark the task and block on nothing; join() marks
// it and then waits out an invocation already inside the handler, so once it returns nothing the
// handler captured is still being read. A join asked for from inside a handler is refused rather
// than served, which is what keeps two strands from waiting on each other. Both verbs are terminal
// and empty the handle, so a caller uses one or the other and never both. The receipt observes its
// scheduler rather than owning it, and outliving that scheduler leaves it inert rather than
// dangling.
class task_handle
{
public:
    task_handle();

    task_handle(const task_handle &)            = delete;
    task_handle &operator=(const task_handle &) = delete;

    task_handle(task_handle &&other) noexcept;
    task_handle &operator=(task_handle &&other) noexcept;

    ~task_handle();

    // A completed one-shot remains valid but is inactive.
    bool valid() const;
    bool active() const;

    // The tallies as of the task's last completed service; a task inside its handler reports the
    // ones it entered with, and a task no longer scheduled reports none.
    task_counters counters() const;

    void cancel();
    expected<void, rejection> join();

private:
    friend class scheduler;

    std::uint64_t m_id;
    std::weak_ptr<pool> m_owner;

    task_handle(const std::shared_ptr<pool> &owner, std::uint64_t id);
};

}

#endif
